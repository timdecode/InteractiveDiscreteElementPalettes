// Copyright 2016 Timothy Davison, all rights reserved.

#pragma once

#include "RuntimeMeshComponent.h"

// Flat shaded mesh factory
// This requires duplicated vertices, so this factory will not share vertices between triangles.
class QuadFactory
{
public:
	TArray<FVector> vertices;
	TArray<FVector> normals;
	TArray<int32> indices;
	TArray<FVector2D> uvs;
	TArray<FRuntimeMeshTangent> tangents;

	void clear()
	{
		vertices.Empty();
		normals.Empty();
		indices.Empty();
		uvs.Empty();
		tangents.Empty();
	}

	static FRuntimeMeshTangent toTangent( FVector vector )
	{
		return FRuntimeMeshTangent( vector.X, vector.Y, vector.Z );
	}

	void pushQuad( FVector& v0, FVector& v1, FVector& v2, FVector& v3, FVector2D uv0 = FVector2D( 0.0f, 0.0f ), FVector2D uv1 = FVector2D( 0.0f, 1.0f ), FVector2D uv2 = FVector2D( 1.0f, 1.0f ), FVector2D uv3 = FVector2D( 1.0f, 0.0f ) )
	{
		pushTriangle( v0, v1, v2, uv0, uv1, uv2 );
		pushTriangle( v0, v2, v3, uv0, uv2, uv3 );
	}

	void pushTriangle( const FVector v0, const FVector v1, const FVector v2, FVector2D u0 = FVector2D::ZeroVector, FVector2D u1 = FVector2D( 0.0f, 1.0f ), FVector2D u2 = FVector2D( 1.0f, 0.0f ) )
	{
		int start = vertices.Num();

		vertices.Add( v0 );
		vertices.Add( v1 );
		vertices.Add( v2 );

		FVector n = FVector::CrossProduct( v2 - v0, v1 - v0 );
		n.Normalize();

		normals.Add( n );
		normals.Add( n );
		normals.Add( n );

		indices.Add( start + 0 );
		indices.Add( start + 1 );
		indices.Add( start + 2 );

		uvs.Add( u0 );
		uvs.Add( u1 );
		uvs.Add( u2 );

		FVector tangentV = FVector::CrossProduct( n, v2 - v0 );
		tangentV.Normalize();

		FRuntimeMeshTangent tangent = toTangent( tangentV );

		tangents.Add( tangent );
		tangents.Add( tangent );
		tangents.Add( tangent );
	}
};

// Doesn't actually smooth or share vertices yet
class SmoothMeshFactory
{
public:
	TArray<FVector> vertices;
	TArray<FVector> normals;
	TArray<int32> indices;
	TArray<FVector2D> uvs;
	TArray<FRuntimeMeshTangent> tangents;

	void clear()
	{
		vertices.Empty();
		normals.Empty();
		indices.Empty();
		uvs.Empty();
		tangents.Empty();
	}

	static FRuntimeMeshTangent toTangent( FVector vector )
	{
		return FRuntimeMeshTangent( vector.X, vector.Y, vector.Z );
	}

	void pushQuad(
		FVector& v0, FVector& v1, FVector& v2, FVector& v3,
		FVector2D uv0 = FVector2D( 0.0f, 0.0f ), FVector2D uv1 = FVector2D( 0.0f, 1.0f ), FVector2D uv2 = FVector2D( 1.0f, 1.0f ), FVector2D uv3 = FVector2D( 1.0f, 0.0f ) )
	{
		pushTriangle( v0, v1, v2, uv0, uv1, uv2 );
		pushTriangle( v0, v2, v3, uv0, uv2, uv3 );
	}

	void pushQuad(
		FVector& v0, FVector& v1, FVector& v2, FVector& v3,
		FVector& n0, FVector &n1, FVector &n2, FVector &n3,
		FVector2D uv0, FVector2D uv1, FVector2D uv2, FVector2D uv3 )
	{
		pushTriangle( v0, v1, v2, n0, n1, n2, uv0, uv1, uv2 );
		pushTriangle( v0, v2, v3, n0, n2, n3, uv0, uv2, uv3 );
	}

	void pushTriangle(
		const FVector& v0, const FVector& v1, const FVector& v2,
		const FVector& n0, const FVector& n1, const FVector& n2,
		const FVector2D u0, const FVector2D u1, const FVector2D u2 )
	{
		int start = vertices.Num();

		vertices.Add( v0 );
		vertices.Add( v1 );
		vertices.Add( v2 );

		normals.Add( n0 );
		normals.Add( n1 );
		normals.Add( n2 );

		indices.Add( start + 0 );
		indices.Add( start + 1 );
		indices.Add( start + 2 );

		uvs.Add( u0 );
		uvs.Add( u1 );
		uvs.Add( u2 );
	}

	void pushTriangle(
		const FVector& v0, const FVector& v1, const FVector& v2 )
	{
		int start = vertices.Num();

		vertices.Add( v0 );
		vertices.Add( v1 );
		vertices.Add( v2 );

		FVector normal = FVector::CrossProduct( v1 - v0, v2 - v0 );

		normals.Add( normal );
		normals.Add( normal );
		normals.Add( normal );

		indices.Add( start + 0 );
		indices.Add( start + 1 );
		indices.Add( start + 2 );

		// calculate non-sheared UV coordinates
		FVector2D u0 = FVector2D( 0.0f, 0.0f );
		FVector2D u1 = FVector2D( 1.0f, 0.0f );
		FVector2D u2 = FVector2D( 5000.0f, 1.0f );

		FVector a = (v1 - v0);
		FVector b = (v2 - v0);

		u2.X = FVector::DotProduct( a, b ) / FVector::DotProduct( a, a );


		uvs.Add( u0 );
		uvs.Add( u1 );
		uvs.Add( u2 );
	}

	void pushTriangle(
		const FVector& v0, const FVector& v1, const FVector& v2,
		const FVector& n0, const FVector& n1, const FVector& n2 )
	{
		int start = vertices.Num();

		vertices.Add( v0 );
		vertices.Add( v1 );
		vertices.Add( v2 );

		normals.Add( n0 );
		normals.Add( n1 );
		normals.Add( n2 );

		indices.Add( start + 0 );
		indices.Add( start + 1 );
		indices.Add( start + 2 );

		// calculate non-sheared UV coordinates
		FVector2D u0 = FVector2D( 0.0f, 0.0f );
		FVector2D u1 = FVector2D( 1.0f, 0.0f );
		FVector2D u2 = FVector2D( 5000.0f, 1.0f );

		FVector a = (v1 - v0);
		FVector b = (v2 - v0);

		u2.X = FVector::DotProduct( a, b ) / FVector::DotProduct( a, a );


		uvs.Add( u0 );
		uvs.Add( u1 );
		uvs.Add( u2 );
	}

	void pushTriangle(
		const FVector v0, const FVector v1, const FVector v2,
		FVector2D u0, FVector2D u1, FVector2D u2 )
	{
		int start = vertices.Num();

		vertices.Add( v0 );
		vertices.Add( v1 );
		vertices.Add( v2 );

		FVector n = FVector::CrossProduct( v2 - v0, v1 - v0 );
		n.Normalize();

		normals.Add( n );
		normals.Add( n );
		normals.Add( n );

		indices.Add( start + 0 );
		indices.Add( start + 1 );
		indices.Add( start + 2 );

		uvs.Add( u0 );
		uvs.Add( u1 );
		uvs.Add( u2 );

		FVector tangentV = FVector::CrossProduct( n, v2 - v0 );
		tangentV.Normalize();

		FRuntimeMeshTangent tangent = toTangent( tangentV );

		tangents.Add( tangent );
		tangents.Add( tangent );
		tangents.Add( tangent );
	}
};