//
//  tcodsMeshInterface.h
//  RegionGrowing
//
//  Created by Timothy Davison on 2015-08-21.
//  Copyright (c) 2015 EpicGames. All rights reserved.
//

#pragma once

#include <unordered_map>

#include "Algorithm/FastBVH/BVH.h"
#include "Algorithm/tcods/HalfEdge.h"

#include "Algorithm/PointCloud.hpp"
#include "Algorithm/nanoflann.hpp"

#include "Utility.h"



struct SamplePoint
{
    float x;
    float y;
    float z;
    uint32_t faceIndex;
};

/**
 An interface for navigating the surface of a UStaticMesh instance within the 
 Region Growing Algorithm. The coordinate space of all operations inside of the
 mesh interface are in world space, as determined by the toWorld transform passed in
 tcodsMeshInterface::buildMesh.
 */
struct tcodsMeshInterface
{
public:
    ~tcodsMeshInterface() { delete _index; }
    
    auto buildMesh( UStaticMeshComponent * uMesh_in, class URuntimeMeshComponent * runtimeMeshToCopyInto, const FTransform& toWorld, FBox sampleLimits = FBox()) -> bool;
    
	void rebuildBvh();

    /** \param uStaticMeshVertex A vertex directly from the UStaticMesh instance used to
     build the mesh interface.
     */
    auto indexOfVertex(const FVector& uStaticMeshVertex, int32& index_out) -> bool;
    
    auto samplePoints() -> std::vector<SamplePoint>;
    
    struct NearestPoint 
	{ 
		FVector point; 
		uint32_t faceIndex; 
	};
    
    auto nearestPointOnMesh(const FVector& pointInWorld) -> NearestPoint;

    auto getIntersection(const FVector& rayOrigin, const FVector& rayDir) -> std::pair<bool, FVector>
    {
        bvh::Ray ray;
        ray.origin = rayOrigin;
        ray.direction = rayDir;
        ray.direction.Normalize();
        
        bvh::IntersectionInfo intersection;
        if( !bvh.getIntersection(ray, intersection, false, true) )
            return std::pair<bool, FVector>(false, FVector::ZeroVector);
        
        FVector hitPoint = intersection.t * ray.direction + ray.origin;
        
        return std::pair<bool, FVector>(true, hitPoint);
    }

    auto getIntersectionAndFace(const FVector& rayOrigin, const FVector& rayDir)
    -> std::tuple<bool, FVector, uint32_t> // hit, point, face index
    {
        bvh::Ray ray;
        ray.origin = rayOrigin;
        ray.direction = rayDir;
        ray.direction.Normalize();
        
        bvh::IntersectionInfo intersection;
        if( !bvh.getIntersection(ray, intersection, false, true) )
            return std::tuple<bool, FVector, uint32_t>(false, FVector::ZeroVector, 0);
        
        FVector hitPoint = intersection.t * ray.direction + ray.origin;
        
        return std::tuple<bool, FVector, uint32_t>(true, hitPoint, intersection.triangle.faceIndex);
    }
    
    inline auto rotationAtSurfacePoint(const FVector& pointInWorld) -> FQuat
    {
        auto pair = rotationAndNormalAtSurfacePoint(pointInWorld);
        
        return pair.first;
    }
    
    inline auto rotationAndNormalAtSurfacePoint(const FVector& pointInWorld) -> std::pair<FQuat, FVector>
    {
        auto nearest = nearestPointOnMesh(pointInWorld);
        
        return rotationAndNormalAtIndex(nearest.faceIndex);
    }
    
    inline auto rotationAndNormalAtIndex(const uint32_t faceIndex) -> std::pair<FQuat, FVector>
    {
        auto face = mesh.faces[faceIndex];
        
        using namespace tcods;
        
        double alpha = face.alpha;
        
        Vector e1, e2; face.frame(e1, e2);
        Vector n = e2 ^ e1;
        Vector u = e1 * cos(alpha) + e2 * sin(alpha);

		// the vector orthogonal to u in the plane of e1, e2
		const double pi_2 = Math::pid() * .5;
		Vector v = e1 * cos( alpha + pi_2 ) + e2 * sin( alpha + pi_2 );
        

        
        FTransform transform = FTransform( unreal(u), unreal(v), unreal(n), FVector::ZeroVector);
        return std::pair<FQuat, FVector>(transform.GetRotation(), unreal(n));
    }
    
    inline auto frameAtNearest(const NearestPoint& nearest) -> std::pair<FVector,FVector>
    {
        auto face = mesh.faces[nearest.faceIndex];
        
        using namespace tcods;
        
        double alpha = face.alpha;
        
        Vector e1, e2;
        face.frame(e1, e2);

        return std::pair<FVector,FVector>(unreal(e1), unreal(e2));
    }
    
public:
    bvh::BVH bvh;
    tcods::Mesh mesh;
    
    unsigned int sampleSubdivisions = 64;

    std::unordered_map<int, std::vector<int32>> _halfEdgeVertex_to_runtimeMeshVertices;


protected:
    auto _initBVH() -> bool;
    auto _initNearestPoints(FBox limits) -> bool;
    void _buildSamplePoints(FBox limits);
    
protected:
    std::unordered_map<FVector,int32> vertexLookup;

    
    typedef nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<float, PointCloud<SamplePoint> > ,
    PointCloud<SamplePoint>,
    3 /* dim */
    > NearestPointIndexAdaptor_t;
    
    PointCloud<SamplePoint> _nearestPointSamples;
    NearestPointIndexAdaptor_t* _index = nullptr;
    
};



