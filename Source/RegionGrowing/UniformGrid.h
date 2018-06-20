// Copyright 2018 Code Monkey Castle, all rights reserved.

#pragma once

#include <vector>

#include "Utility.h"

template<typename ElementType>
class UniformGrid
{
protected:
	FBox _worldBounds;

	// cached properties of the world bounds
	FVector _worldMin = FVector::ZeroVector;
	FVector _worldSize = FVector::ZeroVector;

	FVector _cellSize = FVector::ZeroVector;
	FVector _invCellSize = FVector::ZeroVector;

	FIntVector _dimensions = FIntVector::ZeroValue;

protected:
	void _setup( FIntVector dimensions, FVector cellSize, FVector worldMin )
	{
		_worldMin = worldMin;

		for(int c = 0; c < 3; ++c)
			_worldSize[c] = dimensions[c] * cellSize[c];

		_cellSize = cellSize;
		_invCellSize = FVector( 1.0f / cellSize.X, 1.0f / cellSize.Y, 1.0f / cellSize.Z );
		_dimensions = dimensions;

		_worldBounds = FBox( _worldMin, _worldMin + _worldSize );
	}

public:
	FIntVector dimensions() { return _dimensions; }

	FVector cellSize() { return _cellSize; }
	FBox bounds() { return _worldBounds; }

	std::vector<ElementType> samples;

	static const FIntVector vertexIndices[8];

	void init( FIntVector dimensions, FVector cellSize, FVector worldMin, const std::vector<ElementType>& samples_in )
	{
		_setup( dimensions, cellSize, worldMin );

		if(samples_in.size() == 0)
			samples.resize( _dimensions.X * _dimensions.Y * _dimensions.Z );
		else
			samples = samples_in;
	}

	// ---------------------------------------------------
	// Sample Data Accessors
	// ---------------------------------------------------
	ElementType& operator()( size_t x, size_t y, size_t z )
	{
		return samples[x + y * _dimensions.X + z * _dimensions.X * _dimensions.Y];
	}

	ElementType& operator()( const FIntVector& i )
	{
		return samples[i.X + i.Y * _dimensions.X + i.Z * _dimensions.X * _dimensions.Y];
	}

	ElementType& operator()( size_t i )
	{
		return samples[i];
	}


	ElementType& clamped( int x, int y, int z )
	{
		x = x < 0 ? 0 : x >= _dimensions.X ? _dimensions.X - 1 : x;
		y = y < 0 ? 0 : y >= _dimensions.Y ? _dimensions.Y - 1 : y;
		z = z < 0 ? 0 : z >= _dimensions.Z ? _dimensions.Z - 1 : z;

		return samples[x + y * _dimensions.X + z * _dimensions.X * _dimensions.Y];
	}

	ElementType& clamped( const FIntVector& i )
	{
		const FIntVector ci = clampedIndex( i );

		return samples[ci.X + ci.Y * _dimensions.X + ci.Z * _dimensions.X * _dimensions.Y];
	}

	FIntVector clampedIndex( const FIntVector& index )
	{
		FIntVector clamped;

		clamped.X = FMath::Clamp( index.X, 0, _dimensions.X - 1 );
		clamped.Y = FMath::Clamp( index.Y, 0, _dimensions.Y - 1 );
		clamped.Z = FMath::Clamp( index.Z, 0, _dimensions.Z - 1 );

		return clamped;
	}

	// ---------------------------------------------------
	// Point Transforms
	// ---------------------------------------------------
	FVector toGridSpace( const FVector& worldPoint )
	{
		return (worldPoint - _worldMin) * _invCellSize;
	}

	FVector toReferenceSpace( const FVector& worldPoint )
	{
		FVector gridPoint = toGridSpace( worldPoint );

		FVector cellPoint;
		for(int c = 0; c < 3; ++c)
			cellPoint[c] = FMath::FloorToFloat( gridPoint[c] );

		FVector r = gridPoint - cellPoint;

		return r;
	}

	FIntVector sampleIndex( const FVector& worldPoint )
	{
		FVector gridSpace = toGridSpace( worldPoint );
		FIntVector index;
		for(int c = 0; c < 3; ++c)
			index[c] = int( gridSpace[c] );

		return index;
	}

	FVector samplePoint( const FIntVector& index )
	{
		return Utility::scale( index, _cellSize ) + _worldMin;
	}

	int flatten( const FIntVector& i )
	{
		return i.X + i.Y * _dimensions.X + i.Z * _dimensions.X * _dimensions.Y;
	}

	FIntVector inflate( int i )
	{
		FIntVector vec;

		const int nLayer = _dimensions.X * _dimensions.Y;
		const int remainder = i % nLayer;

		vec.Z = i / (nLayer);
		vec.Y = remainder / _dimensions.X;
		vec.X = remainder % _dimensions.X;

		return vec;
	}


	//	// ---------------------------------------------------
	//	// Interpolation
	//	// ---------------------------------------------------
	float trilinearValueAt( FVector worldPoint )
	{
		FIntVector pi = sampleIndex( worldPoint );

		FVector r = toReferenceSpace( worldPoint );

		float c_00 = clamped( pi.X + 0, pi.Y + 0, pi.Z + 0 ) * (1.0f - r.X) + clamped( pi.X + 1, pi.Y + 0, pi.Z + 0 ) * r.X;
		float c_10 = clamped( pi.X + 0, pi.Y + 1, pi.Z + 0 ) * (1.0f - r.X) + clamped( pi.X + 1, pi.Y + 1, pi.Z + 0 ) * r.X;
		float c_01 = clamped( pi.X + 0, pi.Y + 0, pi.Z + 1 ) * (1.0f - r.X) + clamped( pi.X + 1, pi.Y + 0, pi.Z + 1 ) * r.X;
		float c_11 = clamped( pi.X + 0, pi.Y + 1, pi.Z + 1 ) * (1.0f - r.X) + clamped( pi.X + 1, pi.Y + 1, pi.Z + 1 ) * r.X;

		float c_0 = c_00 * (1.0f - r.Y) + c_10 * r.Y;
		float c_1 = c_01 * (1.0f - r.Y) + c_11 * r.Y;

		float c = c_0 * (1.0f - r.Z) + c_1 * r.Z;

		return c;
	}

	FVector normalAt( FVector worldPoint )
	{
		FVector n;

		FVector dx = FVector::ZeroVector;
		dx.X = _cellSize.X;

		FVector dy = FVector::ZeroVector;
		dy.Y = _cellSize.Y;

		FVector dz = FVector::ZeroVector;
		dz.Z = _cellSize.Z;

		n.X = trilinearValueAt( worldPoint - dx ) - trilinearValueAt( worldPoint + dx );
		n.Y = trilinearValueAt( worldPoint - dy ) - trilinearValueAt( worldPoint + dy );
		n.Z = trilinearValueAt( worldPoint - dz ) - trilinearValueAt( worldPoint + dz );

		return n.GetSafeNormal();
	}

	FVector normalAt( const FIntVector& i )
	{
		FVector n;

		n.X = clamped( i.X - 1, i.Y, i.Z ) - clamped( i.X + 1, i.Y, i.Z );
		n.Y = clamped( i.X, i.Y - 1, i.Z ) - clamped( i.X, i.Y + 1, i.Z );
		n.Z = clamped( i.X, i.Y, i.Z - 1 ) - clamped( i.X, i.Y, i.Z + 1 );

		return n.GetSafeNormal();
	}
};