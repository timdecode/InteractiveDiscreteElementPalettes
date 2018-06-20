//
//  Algorith_Roveri.cpp
//  RegionGrowing
//
//  Created by Timothy Davison on 2018-05-24.
//  Copyright (c) 2018 Timothy Davison. All rights reserved.
//
//  This is an implementation of Roveri et al. (2015) "Example-based Repetitive Structure Synthesis" Eurographics.
//
//  Roveri et al. mention that they solve for x_i and a_i in a Gauss-Seidel manner.
//  Links on Gauss-Seidel:
//   https://www.maa.org/press/periodicals/loci/joma/iterative-methods-for-solving-iaxi-ibi-gauss-seidel-method
//
//  What I think they mean by this is that Roveri et al. solve for x_i and a_i by sequentially updating x_i and then a_i at each 
//  Gradient descent step.
//
//  Roveri et al. don't include the Gaussian window gradients, here is the Mathematica for those:
//
//   g[x_,s_] := Exp[-(x^2) / (s^2)]
//   Simplify[c2 * -2 * g[(mk - ei) - (qk - xj), sigma * c] * g[(mk - ei), delta*c] * g[(x - xj), delta*c]] / .{c->Sqrt[2 + (sigma / delta) ^ 2], c2->Sqrt[Pi * sigma ^ 2] / Sqrt[2 + (sigma / delta) ^ 2] }
//   Simplify[D[-2 * (g[(mk - ei) - (qk - xj), sigma * c] * g[(mk - ei), delta*c] * g[(x - xj), delta*c]), { mk }] / .{c->Sqrt[2 + (sigma / delta) ^ 2], c2->Sqrt[Pi * sigma ^ 2] / Sqrt[2 + (sigma / delta) ^ 2] }]


#include "RegionGrowing.h"

#include "Algorithm/Algorithm_MyRoveri.h"

#define _USE_MATH_DEFINES // for C++  
#include <cmath>  



AlgorithmResult Algorithm_MyRoveri::generate( std::vector<PositionFace>& positions, float radius /*= -1.0f*/, AABB limits /*= AABB() */ )
{
	using namespace Eigen;

	if(!_didInit)
		_initialize();

	AlgorithmResult result;

	beginRound();

	result = seeding( positions, limits );

	// matching
	QuadratureMapping quadratureMapping = matching();

	// merging
	Merging merging(_output, _exemplar);

	merging.gradientDescentMaxIterations = gradientDescentMaxIterations;
	merging.use2DSynthesis = use2DSynthesis;

	// we'll store the results of merging temporarily (this is an async update)
	std::vector<FElement> mergedElements;

	for(auto& e : _output)
	{
		FElement mergedElement = merging.gradientDescent_x_and_a( *e.get(), sigma, quadratureMapping, learningRate );

		mergedElements.push_back( mergedElement );
	}

	// now assign
	std::vector<FElement*> elements = _output.elements();

	for(int i = 0; i < elements.size(); ++i)
	{
		*elements[i] = mergedElements[i];
	}

	_output.rebalance();


	result.modified = elements;

	if(debug_outputTotalSimilarity)
	{
		Similarity similarity( _output, _exemplar );

		float sim = similarity.discreteSimilarityError( quadratureMapping, sigma );

		UE_LOG( LogTemp, Warning, TEXT( "  similairty: %f" ), sim );
	}


	//// sampling control
	//AlgorithmResult sampleResult = _samplingControl( elements, quadratureMapping, limits );

	//result.append( sampleResult );

	// TODO
	// - deal with limits
	// - integrate into URegionGrowingComponent
	
	// rebuild kd trees
	_output.rebalance();

	endRound( result );

	return result;
}

float Algorithm_MyRoveri::_discreteSamplingDensity( std::vector<FElement*>& elements, Eigen::Vector3f x, const float sigma, const float boxHalfExtents )
{
	float sum = 0.0f;

	for(FElement * e : elements)
	{
		float norm_x_minus_e_i = (x - e->position).norm();

		float upper = std::erf( (boxHalfExtents + 2.0f * norm_x_minus_e_i) / sigma );
		float lower = std::erf( (boxHalfExtents - 2.0f * norm_x_minus_e_i) / sigma );

		sum += upper + lower;
	}

	return sum;
}

// Computes this function (in Mathematica language):
// Integrate[ Exp[-((x - xi + s)^2)/(g^2)],{s, x - xi -d,x -xi + d}, Assumptions -> {Element[{g,x,xi,d},Reals],g>0,d > 0}] 
float Algorithm_MyRoveri::_discreteSamplingErrorDensity( Eigen::Vector3f x, Eigen::Vector3f mx, const float sigma, const float boxHalfExtents )
{
	Eigen::Vector3f box( boxHalfExtents, boxHalfExtents, boxHalfExtents );

	std::vector<FElement*> outputNeighbours = _output.nearestInRadius( x, boxHalfExtents );

	std::vector<FElement*> exampleNeighbours = _exemplar.nearestInRadius( mx, boxHalfExtents );

	float outputSum = _discreteSamplingDensity( outputNeighbours, x, sigma, boxHalfExtents );
	float exampleSum = _discreteSamplingDensity( exampleNeighbours, mx, sigma, boxHalfExtents );

	return outputSum - exampleSum;
}

AlgorithmResult Algorithm_MyRoveri::_samplingControl( std::vector<FElement*>& elements, Algorithm_MyRoveri::QuadratureMapping& quadratureMapping, AABB limits )
{
	using namespace Eigen;

	AlgorithmResult result;

	Vector3f halfExtents( samplingControlRadius, samplingControlRadius, samplingControlRadius );

	if(use2DSynthesis)
		halfExtents.z() = 0.0f; 

	std::uniform_real_distribution<float> xDist( -halfExtents.x(), halfExtents.x() );
	std::uniform_real_distribution<float> yDist( -halfExtents.y(), halfExtents.y() );
	std::uniform_real_distribution<float> zDist( -halfExtents.z(), halfExtents.z() );

	const float samplingError = 0.001f;

	Optimization_m matchingStruct( _output, _exemplar, gradientDescentMaxIterations, gradientDescentPrecision, use2DSynthesis );

	for(FElement * e : elements)
	{
		// remove e
		{
			Vector3f x = e->position;

			Vector3f mx = matchingStruct.compute_m( x, sigma, learningRate, nRandomStarts );

			float density = _discreteSamplingErrorDensity( x, mx, sigma, 3.0f * sigma );

			if(density > samplingError )
			{
				// remove
				result.removed.push_back( e );

				break;
			}
		}

		// add new elements
		{
			Vector3f offset( xDist( mt ), yDist( mt ), zDist( mt ) );

			Vector3f x = e->position + offset;

			const bool inLimits = _inBoundary( x, 0.0f ) && limits.contains(x);

			if(!inLimits)
				break;

			Vector3f mx = matchingStruct.compute_m( x, sigma, learningRate, nRandomStarts );

			float density = _discreteSamplingErrorDensity( x, mx, sigma, sampleSpacing );

			
			if(density < -samplingError)
			{
				// add
				FElement * newElement = _copyExemplarToOutput( e, x, nullptr );

				result.generated.push_back( newElement );
			}
		}
	}

	// update the output
	removeElements( result.removed );

	return result;
}

// ------------------------------------------------------------
// Matching
// ------------------------------------------------------------

Algorithm_MyRoveri::QuadratureMapping Algorithm_MyRoveri::matching()
{
	Optimization_m matching( _output, _exemplar, gradientDescentMaxIterations, gradientDescentPrecision, use2DSynthesis );

	matching.logging = debug_OptimizationMLogging;

	AlignedBox3f outputBounds = _output.computeAABB();

	// grow the matching
	float grow = 10.0f * sigma;
	outputBounds.min() -= Vector3f( grow, grow, use2DSynthesis ? 0.0f : grow );
	outputBounds.max() += Vector3f( grow, grow, use2DSynthesis ? 0.0f : grow );


	Vector3f numSamplesf = ((outputBounds.max() - outputBounds.min()) / sampleSpacing);
	Vector3i numSamples;
	for(int c = 0; c < 3; ++c)
		numSamples(c) = std::round( numSamplesf( c ) );

	if(use2DSynthesis) // hacky? oh yes
		numSamples.z() = 0;

	SimpleUniformGrid<Eigen::Vector3f> quadraturePoints;
	{
		const Vector3f start = outputBounds.center() - (numSamples.cast<float>() * 0.5f * sampleSpacing);

		const Vector3f cellSize( sampleSpacing, sampleSpacing, sampleSpacing );

		if(use2DSynthesis)
			numSamples.z() = 1;

		quadraturePoints.init( numSamples, cellSize, start );
	}

	// matching
	Vector3i index = Vector3i::Zero();

	for( index.z() = 0; index.z() < numSamples.z(); index.z()++ )
	{
		for( index.y() = 0; index.y() < numSamples.y(); index.y()++ )
		{
			for( index.x() = 0; index.x() < numSamples.x(); index.x()++ )
			{
				Vector3f q = quadraturePoints.samplePoint( index );

				Vector3f m = matching.compute_m( q, sigma, learningRate, nRandomStarts );

				quadraturePoints( index ) = m;
			}
		}
	}

	return quadraturePoints;
}

AlgorithmResult Algorithm_MyRoveri::seeding( std::vector<PositionFace>& positions, AABB limits )
{
	AlgorithmResult result;

	if( positions.size() == 0 || _exemplar.size() == 0 )
		return result;

	for(PositionFace& pair : positions)
	{
		auto nearest = _output.nearest( pair.position, 3.0f * sigma );

		if(nearest.element)
		{
			// no seed needed
			return result;
		}
	}

	// add an element at the start of the path



	PositionFace& start = positions[0];

	_initializeOutput( start, limits, result );

	return result;
}

float Algorithm_MyRoveri::Similarity::discreteErrorDensityMeasure( const Eigen::Vector3f x, const Eigen::Vector3f mk, float sigma )
{
	using namespace Eigen;

	float distance = 3.0f * sigma;

	const Vector3f halfExtents = Vector3f::Constant( distance );

	std::vector<FElement*> outputNeighbours = _output.nearestInRadius( x, distance );
	std::vector<FElement*> exampleNeighbours = _exemplar.nearestInRadius( mk, distance );

	const float sqrt2_sigma = std::sqrt( 2.0f ) * sigma;

	float outputSum = 0.0f;
	{
		for(FElement * x_i : outputNeighbours)
		{
			for(FElement * x_j : outputNeighbours)
			{
				float aa = Algorithm_MyRoveri::dot_attributes( x_i, x_j );

				Vector3f vec = (x_i->position - x_j->position);

				float gauss = Algorithm_MyRoveri::gaussian( vec, sqrt2_sigma );

				outputSum += (aa * gauss);
			}
		}
	}

	float outputExampleSum = 0.0f;
	{
		for(FElement * e_i : exampleNeighbours)
		{
			Vector3f eVec = mk - e_i->position;

			for(FElement * x_j : outputNeighbours)
			{
				float ab = Algorithm_MyRoveri::dot_attributes( e_i, x_j );

				Vector3f xVec = x - x_j->position;

				Vector3f vec = eVec - xVec;

				float gauss = Algorithm_MyRoveri::gaussian( vec, sqrt2_sigma );

				outputExampleSum += (ab * gauss);
			}
		}
	}

	float exampleSum = 0.0f;
	{
		for(FElement * e_i : exampleNeighbours)
		{
			for(FElement * e_j : exampleNeighbours)
			{
				float bb = Algorithm_MyRoveri::dot_attributes( e_i, e_j );

				Vector3f vec = e_i->position - e_j->position;

				float gauss = Algorithm_MyRoveri::gaussian( vec, sqrt2_sigma );

				exampleSum += (bb * gauss);
			}
		}
	}

	return (std::sqrt(M_PI * std::pow(sigma, 2.0f)) / 2.0f) * (outputSum - (2.0f * outputExampleSum) + exampleSum);
}

float Algorithm_MyRoveri::Similarity::discreteSimilarityError( QuadratureMapping& quadratureMapping, float sigma )
{
	auto& samples = quadratureMapping.samples;

	float sum = 0.0f;

	for(int i = 0; i < samples.size(); ++i)
	{
		const Vector3f mk = samples[i];
		const Vector3i ik = quadratureMapping.inflate( i );
		const Vector3f qk = quadratureMapping.samplePoint( ik );

		sum += discreteErrorDensityMeasure( qk, mk, sigma );
	}

	return sum;
}

Eigen::Vector3f Algorithm_MyRoveri::Optimization_m::gradient_m( const Eigen::Vector3f& m, const Eigen::Vector3f& q, const float sigma )
{
	using namespace Eigen;

	const Vector3f halfExtents = Vector3f::Constant(3.0f * sigma);

	const float twoSigmaSquared = 2.0f * sigma * sigma;

	Vector3f gradient = Vector3f::Zero();

	std::vector<FElement*> exampleNeighbours = _exemplar.nearestInRadius( m, 3.0f * sigma );

	std::vector<FElement*> outputNeighbours = _output.nearestInRadius( q, 3.0f * sigma );

	if(q.norm() < 10.0f)
	{
		float nothing = 5.0f;
	}

	for(FElement * e : exampleNeighbours)
	{
		for (FElement * x : outputNeighbours )
		{
			float a_dot_b = Algorithm_MyRoveri::dot_attributes( e, x );

			Vector3f vec = (e->position - m) - (x->position - q);

			float vecVal = vec.dot( vec );

			float coefficient = std::exp( -(vec.dot( vec )) / twoSigmaSquared );

			gradient = gradient + a_dot_b * vec * coefficient;
		}
	}

	// according to Roveri's impelmentation
	return (- 2.0f * std::sqrt( M_PI / 2.0f ) / sigma) * gradient;
}

Eigen::Vector3f Algorithm_MyRoveri::Optimization_m::gradientDescentOn_m( const Eigen::Vector3f& m_guess, const Eigen::Vector3f& q, const float sigma, const float learningRate )
{
	using namespace Eigen;

	Vector3f m = m_guess;

	if( logging )
		UE_LOG( LogTemp, Warning, TEXT( "gradientDescentOn_m" ) );

	Vector3f m_prev;

	float previous_step_size = 1.0f;

	Similarity similarity( _output, _exemplar );

	for(uint32_t i = 0; i < gradientDescentMaxIterations && previous_step_size > gradientDescentPrecision; ++i)
	{
		m_prev = m;

		auto gradient = gradient_m( m, q, sigma );

		FVector * fVec = reinterpret_cast<FVector*>(&gradient);

		m = m - learningRate * gradient;

		previous_step_size = (m_prev - m).norm();


		if(logging)
		{
			float discreteSimilarity = similarity.discreteErrorDensityMeasure( q, m, sigma );
			UE_LOG( LogTemp, Warning, TEXT( "  S:%f norm:%f stepSize:%f gradient: %s" ), discreteSimilarity, gradient.norm(), previous_step_size, *fVec->ToString() );
		}
	}

	return m;
}

Eigen::Vector3f Algorithm_MyRoveri::Optimization_m::compute_m( const Eigen::Vector3f& q, const float sigma, const float learningRate, const size_t nStarts )
{
	using namespace Eigen;

	Vector3f bestM;

	AlignedBox3f exemplarBounds = _exemplarBounds;

	// inset the bounds
	//float inset = 3.0f * sigma;

	//exemplarBounds.min() += Vector3f( inset, inset, use2DSynthesis ? 0.0f : inset );
	//exemplarBounds.max() -= Vector3f( inset, inset, use2DSynthesis ? 0.0f : inset );

	std::uniform_real_distribution<float> xDist( exemplarBounds.min().x(), exemplarBounds.max().x() );
	std::uniform_real_distribution<float> yDist( exemplarBounds.min().y(), exemplarBounds.max().y() );
	std::uniform_real_distribution<float> zDist( exemplarBounds.min().z(), exemplarBounds.max().z() );



	for(int i = 0; i < nStarts; ++i)
	{
		Vector3f guess_m( xDist(mt), yDist(mt), zDist(mt));

		Vector3f m = gradientDescentOn_m( guess_m, q, sigma, learningRate );

		// hack, we actually need the total energy here
		bestM = m;
	}

	return bestM;
}

// ------------------------------------------------------------
// Merging
// ------------------------------------------------------------

Eigen::Vector3f Algorithm_MyRoveri::Merging::gradient_x( FElement& element_i, const float sigma, Algorithm_MyRoveri::QuadratureMapping& quadratureMapping )
{
	using namespace Eigen;

	Vector3f x_i = element_i.position;

	const float maxDistance = 3.0f * sigma; // according to the paper
	Vector3f halfExtents( maxDistance, maxDistance, maxDistance );

	Vector3f gradient = Vector3f::Zero();

	Vector3f start = x_i - halfExtents;
	Vector3f end = x_i + halfExtents;

	Vector3i startI = quadratureMapping.clampedIndex( quadratureMapping.sampleIndex( start ) );
	Vector3i endI = quadratureMapping.clampedIndex( quadratureMapping.sampleIndex( end ) );

	Vector3i index;

	const float sqrt2_sigma = std::sqrt( 2.0f ) * sigma;

	float suffix = -1.0f / (2.0f * sigma * sigma);
	float prefix = -1.0f * std::sqrt( M_PI / 2.0f ) / sigma;

	for(index.z() = startI.z(); index.z() <= endI.z(); index.z()++)
	{
		for(index.y() = startI.y(); index.y() <= endI.y(); index.y()++)
		{
			for(index.x() = startI.x(); index.x() <= endI.x(); index.x()++)
			{
				Vector3f q_k = quadratureMapping.samplePoint( index );
				Vector3f m_k = quadratureMapping( index );

				Vector3f dist_a = q_k - m_k - x_i;

				// the example sum term
				Vector3f exampleSum = Vector3f::Zero();
				{
					std::vector<FElement*> exampleNeighbours = _exemplar.nearestInRadius( m_k, maxDistance );

					for(FElement * element_j : exampleNeighbours)
					{
						Vector3f& e_j = element_j->position;

						Vector3f dist = e_j + dist_a;

						float distV = dist.dot( dist );

						float gauss = std::exp( distV * suffix );

						exampleSum = exampleSum + dist * gauss;
					}
				}

				// the output sum term
				Vector3f outputSum = Vector3f::Zero();
				{
					std::vector<FElement*> outputNeighbours = _output.nearestInRadius( q_k, maxDistance );

					for(FElement * element_j : outputNeighbours)
					{
						Vector3f& x_j = element_j->position;

						float ab = Algorithm_MyRoveri::dot_attributes( &element_i, element_j );

						Vector3f vec = (x_i - x_j);

						float distV = vec.dot( vec );

						float gauss = std::exp( distV * suffix );

						outputSum = outputSum + vec * gauss;
					}
				}

				gradient = gradient + prefix * (exampleSum + outputSum);
			}
		}
	}

	return gradient;
}

FElement Algorithm_MyRoveri::Merging::gradientDescent_x_and_a( FElement& guess, const float sigma, Algorithm_MyRoveri::QuadratureMapping& quadratureMapping, float learningRate )
{
	using namespace Eigen;

	FElement e = guess;

	for(uint32_t i = 0; i < gradientDescentMaxIterations; ++i)
	{
		auto gradient = gradient_x( e, sigma, quadratureMapping );

		if(use2DSynthesis)
			gradient.z() = 0.0f;

		e.position = e.position - learningRate * gradient;



		// this is where we would do the attributes next, in a Gauss-Seidel manner
	}

	return e;
}

// ------------------------------------------------------------
// Shared
// ------------------------------------------------------------

float Algorithm_MyRoveri::gaussian( Eigen::Vector3f v, float sigma )
{
	return std::exp( -1.0f * (v.squaredNorm() / std::pow( sigma, 2.0f )) );
}

float Algorithm_MyRoveri::dot_attributes( FElement * a, FElement * b )
{
	return (a->type == b->type ? 1.0f : 0.0f);
}

#undef _USE_MATH_DEFINES


