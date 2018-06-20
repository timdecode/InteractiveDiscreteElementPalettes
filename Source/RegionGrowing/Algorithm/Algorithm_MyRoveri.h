//
//  Algorith_Roveri.h
//  RegionGrowing
//
//  Created by Timothy Davison on 2018-05-24.
//  Copyright (c) 2018 Timothy Davison. All rights reserved.
//
//  This is an implementation of Roveri et al. (2015) "Example-based Repetitive Structure Synthesis" Eurographics.
//

#pragma once

#include "Algorithm/Algorithm.h"
#include "Algorithm/SimpleUniformGrid.h"

#include <random>
#include <iostream>

class Algorithm_MyRoveri : public Algorithm
{
public:
	float sampleSpacing = 1.0f;

	float sigma = 0.5f;

	float learningRate = 0.1f;

	uint32_t nRandomStarts = 5;

	bool use2DSynthesis = true;

	float samplingControlRadius = 0.5f;

	bool debug_outputTotalSimilarity = false;

	int gradientDescentMaxIterations = 50;
	float gradientDescentPrecision = 0.001f;

	bool debug_OptimizationMLogging = false;

public:
	Algorithm_MyRoveri()
	{
		std::random_device rd;
		mt = std::mt19937( rd() );
	}

	virtual ~Algorithm_MyRoveri() {}

	virtual AlgorithmResult generate( std::vector<PositionFace>& positions, float radius = -1.0f, AABB limits = AABB() );

	typedef SimpleUniformGrid<Eigen::Vector3f> QuadratureMapping;

	QuadratureMapping matching();

	AlgorithmResult seeding( std::vector<PositionFace>& positions, AABB limits );



private:
	float _discreteSamplingDensity( std::vector<FElement*>& elements, Eigen::Vector3f x, const float sigma, const float boxHalfExtents );
	float _discreteSamplingErrorDensity( Eigen::Vector3f x, Eigen::Vector3f mx, const float sigma, const float boxHalfExtents );
	
	AlgorithmResult _samplingControl( std::vector<FElement*>& elements, Algorithm_MyRoveri::QuadratureMapping& quadratureMapping, AABB limits );

	struct Similarity
	{
	public:
		Similarity( Domain& output, Domain& exemplar ) : _output( output ), _exemplar( exemplar )
		{

		}

		float discreteErrorDensityMeasure( const Eigen::Vector3f x, const Eigen::Vector3f mk, float sigma );
		float discreteSimilarityError( Algorithm_MyRoveri::QuadratureMapping& quadratureMapping, float sigma );

	protected:
		Domain& _output;
		Domain& _exemplar;
	};

	struct Optimization_m
	{
	public:
		Optimization_m(
			Domain& output, 
			Domain& exemplar,
			uint32_t gradientDescentMaxIterations,
			float gradientDescentPrecision,
			bool use2DSynthesis
		) : _output(output), _exemplar(exemplar), gradientDescentMaxIterations(gradientDescentMaxIterations), gradientDescentPrecision(gradientDescentPrecision), use2DSynthesis(use2DSynthesis) 
		{
			_exemplarBounds = _exemplar.computeAABB();
			_outputBounds = _output.computeAABB();

			std::random_device rd;
			mt = std::mt19937( rd() );
		}



		Eigen::Vector3f gradient_m( const Eigen::Vector3f& m, const Eigen::Vector3f& q, const float sigma );

		Eigen::Vector3f gradientDescentOn_m( const Eigen::Vector3f& m_guess, const Eigen::Vector3f& q, const float sigma, const float learningRate );

		Eigen::Vector3f compute_m( const Eigen::Vector3f& q, const float sigma, const float learningRate, const size_t nStarts = 5 );

	public:
		uint32_t gradientDescentMaxIterations = 5000;
		bool use2DSynthesis = false;
		float gradientDescentPrecision = 0.01f;
		bool logging = false;

	protected:
		Domain& _output;
		Domain& _exemplar;

		Eigen::AlignedBox3f _outputBounds;
		Eigen::AlignedBox3f _exemplarBounds;

		std::mt19937 mt;
	};



	struct Merging
	{
	public:
		Merging( Domain& output, Domain& exemplar ) : _output( output ), _exemplar( exemplar )
		{
			_exemplarBounds = _exemplar.computeAABB();
			_outputBounds = _output.computeAABB();
		}

		FElement gradientDescent_x_and_a( FElement& element_i, const float sigma, Algorithm_MyRoveri::QuadratureMapping& quadratureMapping, float learningRate );

		Eigen::Vector3f gradient_x( FElement& element_i, const float sigma, Algorithm_MyRoveri::QuadratureMapping& quadratureMapping );

	public:
		uint32_t gradientDescentMaxIterations = 50;

		bool use2DSynthesis = false;


	protected:
		Domain& _output;
		Domain& _exemplar;

		Eigen::AlignedBox3f _outputBounds;
		Eigen::AlignedBox3f _exemplarBounds;
	};

	static float gaussian( Eigen::Vector3f v, float sigma );

	static float dot_attributes( FElement * a, FElement * b );

protected:
	std::mt19937 mt;
};