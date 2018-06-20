//
//  tcodsMeshInterface.cpp
//  RegionGrowing
//
//  Created by Timothy Davison on 2015-08-21.
//  Copyright (c) 2015 EpicGames. All rights reserved.
//

#include "RegionGrowing.h"
#include "tcodsMeshInterface.h"

#include "Algorithm/tcods/HalfEdge.h"
#include "Algorithm/tcods/MeshIO.h"
#include "RuntimeMeshComponent.h"
#include "RuntimeMeshLibrary.h"


#include "StaticMeshResources.h"
#include "Utility.h"


#include <vector>



auto tcodsMeshInterface::indexOfVertex(const FVector& uStaticMeshVertex, int32& index_out) -> bool
{
	auto found = vertexLookup.find( uStaticMeshVertex );

    if(found == vertexLookup.end() )
        return false;
    
	index_out = found->second;
    
    return true;
}

auto tcodsMeshInterface::buildMesh( UStaticMeshComponent* meshComponent, URuntimeMeshComponent * runtimeMesh, const FTransform& toWorld, FBox limits) -> bool
{
	using namespace tcods;

    mesh = Mesh();
    
    vertexLookup.clear();
	_halfEdgeVertex_to_runtimeMeshVertices.clear();
    
    UStaticMesh& uMesh = *meshComponent->GetStaticMesh();

    if(uMesh.RenderData == nullptr ) return false;
    if(uMesh.RenderData->LODResources.Num() == 0 ) return false;


	// build the TCODS half-edge data structure
	MeshIO::MeshData data;
	{
		auto& positions = data.positions;
		auto& indices = data.indices;
		auto& normals = data.normals;

		FStaticMeshLODResources& resource = uMesh.RenderData->LODResources[meshComponent->PreviousLODLevel];

		FPositionVertexBuffer& positionBuffer = resource.PositionVertexBuffer;
		FRawStaticIndexBuffer& indexBuffer = resource.IndexBuffer;
		FStaticMeshVertexBuffer& vertexBuffer = resource.VertexBuffer;

		// now we need to process it with tcods
		using namespace tcods;

		uint32 vCount = positionBuffer.GetNumVertices();

		// Unreal stores duplicate vertices, we need to resolve this with a map
		std::vector<uint32> vertexIndices;
		vertexIndices.resize( vCount );

		// but each face has its own set of normals for each vertex in the face

		for(uint32 i = 0; i < vCount; ++i)
		{
			const FVector& v = positionBuffer.VertexPosition( i );
			const auto& normal = vertexBuffer.VertexTangentZ( i );

			normals.push_back( to_tcods( normal ) );

			auto found = vertexLookup.find( v );
			if(found == vertexLookup.end())
			{
				vertexLookup[v] = positions.size();

				vertexIndices[i] = positions.size();

				positions.push_back( to_tcods( toWorld.TransformPosition( v ) ) );
			}
			else
				vertexIndices[i] = found->second;
		}

		FIndexArrayView indexArrayView = indexBuffer.GetArrayView();
		int32 iCount = indexArrayView.Num();
		for(int32 i = 0; i + 2 < iCount; i += 3)
		{
			// let's assume triangles
			std::vector<MeshIO::Index> face( 3 );

			// need to consider the winding order wrt to the face normal
			int32 indexArrayView0 = indexArrayView[i + 0];
			int32 indexArrayView1 = indexArrayView[i + 1];
			int32 indexArrayView2 = indexArrayView[i + 2];

			tcods::Vector n0 = normals[indexArrayView0];
			tcods::Vector n1 = normals[indexArrayView1];
			tcods::Vector n2 = normals[indexArrayView2];

			tcods::Vector n = (n0 + n1 + n2) / 3.0;

			int32 i0 = vertexIndices[indexArrayView[i + 0]];
			int32 i1 = vertexIndices[indexArrayView[i + 1]];
			int32 i2 = vertexIndices[indexArrayView[i + 2]];

			face[0] = MeshIO::Index( i0, -1, indexArrayView[i + 0] );
			face[1] = MeshIO::Index( i1, -1, indexArrayView[i + 1] );
			face[2] = MeshIO::Index( i2, -1, indexArrayView[i + 2] );




			indices.push_back( face );
		}

		MeshIO::buildMesh( data, mesh );

		mesh.topologyChange();
		mesh.geometryChange();
	}


	// create a runtime static mesh directly from the tcods half-edge data structure so that our face indices are in sync
	{
		TArray<FVector> runtimePositions;
		TArray<FRuntimeMeshVertexNoPosition> runtimeVertexData;
		TArray<int32> runtimeTriangles;

		const FTransform invToWorld = toWorld.Inverse();

		runtimeMesh->ClearAllMeshSections();

		// faces in tcods are ordered already by face.index
		int32 fi = 0;
		for(auto& face : mesh.faces)
		{
			auto halfEdge = face.he;

			if(face.isBoundary())
				continue;

			int ti = 0;
			do 
			{
				FVector position = unreal(halfEdge->from->position);

				position = invToWorld.TransformPosition( position );

				_halfEdgeVertex_to_runtimeMeshVertices[halfEdge->from->index].push_back( runtimePositions.Num() );

				runtimePositions.Add( position );

				tcods::Vector e1, e2;
				face.frame( e1, e2 );
				
				FRuntimeMeshVertexNoPosition vertex( unreal( e1 ), unreal( e2 ), unreal( face.normal() ) );
				runtimeVertexData.Add( vertex );



				halfEdge = halfEdge->next;
				ti++;

			} while (halfEdge != face.he);

			assert( ti <= 3 );

			runtimeTriangles.Add( fi++ );
			runtimeTriangles.Add( fi++ );
			runtimeTriangles.Add( fi++ );
		}

		runtimeMesh->CreateMeshSectionDualBuffer( 0, runtimePositions, runtimeVertexData, runtimeTriangles, true, EUpdateFrequency::Frequent );
		runtimeMesh->SetMaterial( 0, meshComponent->GetMaterial( 0 ) );
	}

    if( !_initBVH() ) return false;
    if( !_initNearestPoints(limits) ) return false;
    
    return true;
}

void tcodsMeshInterface::rebuildBvh()
{
	_initBVH();
}

auto tcodsMeshInterface::_initBVH() -> bool
{
    std::vector<bvh::Triangle> triangles;
    triangles.reserve(mesh.faces.size());
    
    uint32_t index = 0;
    for( const tcods::Face& face : mesh.faces )
    {
        auto a = face.he->from->position;
        auto b = face.he->next->from->position;
        auto c = face.he->next->next->from->position;
        
        triangles.emplace_back();
        bvh::Triangle& t = triangles.back();
        t.vertices = {{unreal(a), unreal(b), unreal(c)}};
        t.faceIndex = index;
        
        ++index;
    }
    
    bvh.build(triangles);
    
    return true;
}

auto tcodsMeshInterface::samplePoints() -> std::vector<SamplePoint>
{
    return _nearestPointSamples.pts;
}



void tcodsMeshInterface::_buildSamplePoints(FBox limits)
{
    _nearestPointSamples.pts.clear();
    
    FBox bounds = FBox(EForceInit::ForceInitToZero);
    
    for( const tcods::Vertex& vertex : mesh.vertices )
    {
        FVector fPosition = unreal(vertex.position);
        
        bounds += fPosition;
    }
    
    FVector size = bounds.GetSize();
    float max = size.GetMax();
    
    float stepSize = max / sampleSubdivisions;
    
    bvh::Ray ray;
    
    std::vector<SamplePoint>& hits = _nearestPointSamples.pts;
    
    for( unsigned int dim = 0; dim < 3; ++dim )
    {
        unsigned int dimI = (dim + 1) % 3;
        unsigned int dimJ = (dim + 2) % 3;
        
        ray.direction = FVector::ZeroVector;
        ray.direction[dim] = 1.0f;
        
        ray.origin[dim] = bounds.Min[dim];
        
        for( float i = bounds.Min[dimI] - 2.0f * stepSize; i <= bounds.Max[dimI] + stepSize; i += stepSize )
        {
            ray.origin[dimI] = i;
            
            for( float j = bounds.Min[dimJ] - 2.0f * stepSize; j < bounds.Max[dimJ] + stepSize; j+= stepSize )
            {
                ray.origin[dimJ] = j;
                
                auto intersections = bvh.getIntersections(ray, false);
                
                for( auto& intersection : intersections )
                {
                    const FVector p = intersection.t * ray.direction + ray.origin;

					if( limits.IsValid && limits.IsInside(p) )
						hits.push_back({p.X, p.Y, p.Z, intersection.triangle.faceIndex});
                }
            }
        } 
    }
}

auto tcodsMeshInterface::_initNearestPoints(FBox limits) -> bool
{
    if( _index ) { delete _index; _index = nullptr; }
    
    _buildSamplePoints(limits);
    
    _index = new NearestPointIndexAdaptor_t(3, _nearestPointSamples, nanoflann::KDTreeSingleIndexAdaptorParams(2));
    _index->buildIndex();
    
    return true;
}

auto tcodsMeshInterface::nearestPointOnMesh(const FVector& point) -> NearestPoint
{
    if( !_index || _nearestPointSamples.pts.size() == 0 )
        return { FVector::ZeroVector, 0 };
    
    float kdPoint[] = {point.X, point.Y, point.Z};
    
    float distance;
    size_t index;
    
    _index->knnSearch(&kdPoint[0], 1, &index, &distance);
    
    SamplePoint& nearestSample = _nearestPointSamples.pts[index];
    
    const auto& mainFace = mesh.faces[nearestSample.faceIndex];
    
    FVector a = unreal(mainFace.he->from->position);
    FVector b = unreal(mainFace.he->next->from->position);
    FVector c = unreal(mainFace.he->next->next->from->position);
    
    FVector nearestPoint = FMath::ClosestPointOnTriangleToPoint(point, a, c, b);
    float nearestDistance = FVector::DistSquared(point, nearestPoint);
	uint32_t nearestFace = nearestSample.faceIndex;

    auto mainHe = mainFace.he;
    
    do
    {
        auto faceEdge = mainHe;
        
        do
        {
            a = unreal(faceEdge->from->position);
            b = unreal(faceEdge->next->from->position);
            c = unreal(faceEdge->next->next->from->position);
            
            FVector facePoint = FMath::ClosestPointOnTriangleToPoint(point, a, c, b);
            
            float distance = FVector::DistSquared(point, facePoint);
            
            if( distance < nearestDistance )
            {
                nearestPoint = facePoint;
                nearestDistance = distance;
				nearestFace = faceEdge->face->index;
            }
            
            faceEdge = faceEdge->next->flip;
        } while( faceEdge != mainHe );
        
        mainHe = mainHe->next;
    } while( mainHe != mainFace.he );
    
    NearestPoint nearest;
	nearest.faceIndex = nearestFace; 
    nearest.point = nearestPoint;
    
    return nearest;
}

