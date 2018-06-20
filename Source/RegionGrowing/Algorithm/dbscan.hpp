//
//  dbscan.hpp
//  RegionGrowing
//
//  Created by Timothy Davison on 2016-01-01.
//  Copyright © 2016 Timothy Davison, Inc. All rights reserved.
//

#pragma once

#include <vector>

#include "Algorithm/Element.h"
#include "Algorithm/Domain.h"
#include "Algorithm/ElementKDTree.h"

namespace rg
{
	struct PointAndIndex
	{
		PointAndIndex( Eigen::Vector3f p, size_t i ) : point( p ), index( i ) {}

		Eigen::Vector3f point;
		size_t index;
	};

    void dbscan(std::vector<Eigen::Vector3f>& points,
                float radius,
                int minPoints,
                std::vector<std::vector<PointAndIndex>>& clusters_out,
                std::vector<PointAndIndex>& noise_out)
    {
        clusters_out.clear();
        noise_out.clear();
        
        if( points.size() == 0 )
            return;

        float radiusSqrd = radius * radius;
        
        EigenPointCloud kdPoints;
        
        EigenDynamicPointCloudIndexAdaptor kdTree(3, kdPoints, nanoflann::KDTreeSingleIndexAdaptorParams(10 /* max leaf */));

		kdPoints.pts = points;
		if(kdPoints.pts.size() > 0)
			kdTree.addPoints( 0, kdPoints.pts.size() - 1 );
//        kdTree.buildIndex();

        nanoflann::SearchParams searchParams;
        searchParams.sorted = false;
        
        std::unordered_set<size_t> unvisited;
        for( int i = 0; i < points.size(); ++i )
            unvisited.insert(i);

        std::unordered_set<size_t> clustered;
        
        while( unvisited.size() )
        {
            size_t i = *unvisited.begin();
            unvisited.erase(i);
            
            Eigen::Vector3f& p = points[i];
            
            std::vector<std::pair<size_t, float> > neighbours;
            kdTree.radiusSearch(&p(0), radiusSqrd, neighbours, searchParams);

            // -1 because we include the query point
            if( neighbours.size() - 1 < minPoints )
				noise_out.emplace_back( p, i );
            else
            {
                clusters_out.emplace_back();
                auto& cluster = clusters_out.back();

                // expand cluster
				cluster.emplace_back( p, i );
                clustered.insert(i);
                
                for( auto& pair : neighbours )
                {
                    size_t i_ = pair.first;
                    Eigen::Vector3f& p_ = points[i_];
                    
                    if( unvisited.find(i_) != unvisited.end() )
                    {
                        unvisited.erase(i_);
                        
                        std::vector<std::pair<size_t, float> > neighbours_;
                        kdTree.radiusSearch(&p_(0), radiusSqrd, neighbours, searchParams);
                        
                        // -1 because we include the query point
                        if( neighbours_.size() - 1>= minPoints )
                            neighbours.insert(neighbours.end(), neighbours_.begin(), neighbours_.end());
                        
                        if( clustered.find(i_) == clustered.end() )
							cluster.emplace_back( p_,i_ );
                    }
                }
            }
        }
    }
};