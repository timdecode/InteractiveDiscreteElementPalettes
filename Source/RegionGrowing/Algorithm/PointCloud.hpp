//
//  PointCloud.hpp
//  RegionGrowing
//
//  Created by Timothy Davison on 2015-09-08.
//  Copyright © 2015 EpicGames. All rights reserved.
//

#pragma once

#include "Eigen/Dense"
#include "Algorithm/nanoflann.hpp"

struct EigenPointCloud
{
    std::vector<Eigen::Vector3f>  pts;
    
    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return pts.size(); }
    
    // Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
    inline float kdtree_distance(const float *p1, const size_t idx_p2,size_t /*size*/) const
    {
        const auto& p2_3f = pts[idx_p2];
        const auto& p1_3f = *reinterpret_cast<const Eigen::Vector3f*>(p1);
        
        return (p2_3f - p1_3f).squaredNorm();
    }
    
    // Returns the dim'th component of the idx'th point in the class:
    // Since this is inlined and the "dim" argument is typically an immediate value, the
    //  "if/else's" are actually solved at compile time.
    inline float kdtree_get_pt(const size_t idx, int dim) const
    {
        return pts[idx](dim);
    }
    
    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
    //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /*bb*/) const { return false; }
};



// dynamic adapter
typedef nanoflann::KDTreeSingleIndexDynamicAdaptor<
	nanoflann::L2_Simple_Adaptor<float, EigenPointCloud>,
EigenPointCloud,
3 /* dim */
> EigenDynamicPointCloudIndexAdaptor;

template <typename T>
struct PointCloudPoint
{
    T  x,y,z;
};

template <typename PointType>
struct PointCloud
{
    std::vector<PointType>  pts;
    
    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return pts.size(); }
    
    // Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
    inline decltype(PointType().x) kdtree_distance(const decltype(PointType().x) *p1, const size_t idx_p2,size_t /*size*/) const
    {
        const auto d0=p1[0]-pts[idx_p2].x;
        const auto d1=p1[1]-pts[idx_p2].y;
        const auto d2=p1[2]-pts[idx_p2].z;
        return d0*d0+d1*d1+d2*d2;
    }
    
    // Returns the dim'th component of the idx'th point in the class:
    // Since this is inlined and the "dim" argument is typically an immediate value, the
    //  "if/else's" are actually solved at compile time.
    inline decltype(PointType().x) kdtree_get_pt(const size_t idx, int dim) const
    {
        if (dim==0) return pts[idx].x;
        else if (dim==1) return pts[idx].y;
        else return pts[idx].z;
    }
    
    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
    //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /*bb*/) const { return false; }
    
};