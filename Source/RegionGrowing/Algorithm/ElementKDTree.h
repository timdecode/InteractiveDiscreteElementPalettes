//
//  ElementKDTree.h
//  RegionGrowing
//
//  Created by Timothy Davison on 2015-12-01.
//  Copyright © 2015 Epic Games, Inc. All rights reserved.
//

#pragma once

#include <vector>
#include <iostream>
#include <memory>
#include <unordered_set>

#include "Element.h"
#include "PointCloud.hpp"

typedef nanoflann::KDTreeSingleIndexDynamicAdaptor<
nanoflann::L2_Simple_Adaptor<float, PointCloud<PointCloudPoint<float> > > ,
PointCloud<PointCloudPoint<float> >,
3 /* dim */
> IndexAdaptor_t;

struct ElementKDTree
{
public:
    ElementKDTree()
    {
        _setupIndex();
    }
    
    ~ElementKDTree()
    {
        delete _index;
    }
    
    // Add the element without claiming ownership over the pointer
    void add(FElement * element)
    {
        size_t newIndex = _cloud.kdtree_get_point_count();

        const auto& p = element->position;
        _cloud.pts.emplace_back(PointCloudPoint<float>{p.x(), p.y(), p.z()});
        
        _elements.push_back(element);

		_index->addPoints( newIndex, newIndex );
    }
    
    size_t size()
    {
        return _elements.size();
    }
    
    template<typename Container>
    void setElements(Container& elements)
    {
        clear();

		_setupIndex();

        
        for( FElement * element : elements )
        {
            const auto p = element->position;
            
            _cloud.pts.emplace_back(PointCloudPoint<float>{ p.x(), p.y(), p.z() });
            _elements.push_back(element);
        }
        
		if( _cloud.pts.size() > 0 )
			_index->addPoints( 0, _cloud.pts.size() - 1 );
    }

    void clear()
    {
        _cloud.pts.clear();
        _elements.clear();
        
        _setupIndex();
    }
    
    struct NearestResult
    {
        FElement * element;
        float distanceSquared;
    };
    
    // queries
    NearestResult nearest(const Eigen::Vector3f& point)
    {
        if( _elements.size() == 0 )
            return {nullptr, 0.0f};
        
        bool found = false;
        
        float distanceSquared;
        size_t index;
        
        _index->knnSearch(&point(0), 1, &index, &distanceSquared);
        
        return {_elements[index], distanceSquared};
    }
    
    std::vector<FElement*> nearestKNeighbours(const Eigen::Vector3f& point, size_t count, std::vector<float>& distances_out)
    {
        if( count == 0 )
            return std::vector<FElement*>();
        
        if( count > _elements.size() )
            count = _elements.size();
        
        std::vector<FElement*> result;
        
        if( _elements.size() == 0 )
            return result;
        
        result.reserve(count);
        
        std::vector<size_t> indices(count);
        std::vector<float> distances(count);
        
        nanoflann::KNNResultSet<float,size_t> resultSet(count);
        resultSet.init(&indices[0], &distances[0]);
        _index->findNeighbors(resultSet, &point(0), nanoflann::SearchParams());
        
        for( int i = 0; i < resultSet.size(); ++i )
        {
            FElement * element = _elements[indices[i]];
            
            result.emplace_back(element);
            distances_out.push_back(distances[i]);
        }
        
        return result;
    }
    
    std::vector<FElement*> nearestKNeighbours(const Eigen::Vector3f& point, size_t count)
    {
        if( count == 0 )
            return std::vector<FElement*>();
        
        if( _elements.size() == 0 )
            return std::vector<FElement*>();
        
        if( count > _elements.size() )
            count = _elements.size();
        
        std::vector<size_t> indices(count);
        std::vector<float> distances(count);
        
        nanoflann::KNNResultSet<float,size_t> resultSet(count);
        resultSet.init(&indices[0], &distances[0]);
        _index->findNeighbors(resultSet, &point(0), nanoflann::SearchParams());
        
        std::vector<FElement*> result(resultSet.size());

        for( int i = 0; i < resultSet.size(); ++i )
        {
            FElement * element = _elements[indices[i]];
            
            result[i] = element;
        }
        
        return result;
    }
    
    std::vector<FElement*> nearestInRadius(const Eigen::Vector3f& point, float radius, std::vector<float>& distances_out, int count = -1, bool sort = true)
    {
        if( _elements.size() == 0 )
            return std::vector<FElement*>();
        
        if( radius <= 0.0f )
            return std::vector<FElement*>();
        
        std::vector<std::pair<size_t, float> > dynoResult;
        
        nanoflann::SearchParams searchParams;
        
        searchParams.sorted = sort;
        
        _index->radiusSearch(&point(0), radius * radius, dynoResult, searchParams);

        std::vector<FElement*> result(dynoResult.size());
        
        size_t i = 0;
        for( auto pair : dynoResult )
        {
            FElement * element = _elements[pair.first];

            result[i] = element;
            ++i;

            distances_out.push_back(pair.second);
        }
        
        return result;
    }
    
    std::vector<FElement*> nearestInRadius(const Eigen::Vector3f& point, float radius, int count = -1, bool sort = true)
    {
        if( _elements.size() == 0 )
            return std::vector<FElement*>();

        if( radius <= 0.0f )
            return std::vector<FElement*>();
        
        std::vector<std::pair<size_t, float> > dynoResult;
        
        nanoflann::SearchParams searchParams;
        
        searchParams.sorted = sort;
        
        _index->radiusSearch(&point(0), radius * radius, dynoResult, searchParams);
        
        std::vector<FElement*> result(dynoResult.size());
        
        size_t i = 0;
        for( auto& pair : dynoResult )
        {
            FElement * element = _elements[pair.first];
            
            result[i] = element;
            
            ++i;
        }
        
        return result;
    }
    
    std::vector<FElement*> neighbours(FElement * element, float radius, int count = -1, bool sort = true, std::vector<float>* distances_out = nullptr, bool surfaceVolumeInteraction  = true)
    {
        
        if( _elements.size() == 0 )
            return std::vector<FElement*>();
        
        if( radius <= 0.0f )
            return std::vector<FElement*>();
        
        std::vector<std::pair<size_t, float> > dynoResult;
        
        nanoflann::SearchParams searchParams;
        
        searchParams.sorted = sort;
        
        _index->radiusSearch(&element->position(0), radius * radius, dynoResult, searchParams);
        
        if( distances_out != nullptr )
            distances_out->clear();
        
		if(dynoResult.size() == 0)
			return std::vector<FElement*>();

		std::vector<FElement*> result;
		result.reserve(dynoResult.size() - 1);
        
        for( auto& pair : dynoResult )
        {
            FElement * e = _elements.at(pair.first);
            
            if( e == element )
                continue;

			// don't allow surface elements to interact with volume elements
			if(!surfaceVolumeInteraction && element->faceIndex > 0 && e->faceIndex <= 0)
				continue;

            result.push_back(e);
            
            if( distances_out != nullptr )
                distances_out->push_back(pair.second);
        }
        
        return result;
    }
    
    void rebalance()
    {
        _updatePositions();

	
	}
    
    auto begin() -> decltype( std::vector<FElement*>().begin() )
    {
        return _elements.begin();
    }
    
    auto end() -> decltype( std::vector<FElement*>().begin() )
    {
        return _elements.end();
    }
    
private:
    void _setupIndex()
    {
        if( _index )
            delete _index;
        
        _index = new IndexAdaptor_t(3, _cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10 /* max leaf */));
		
//        _index->buildIndex();
    }
    
    void _updatePositions()
    {
		std::vector<PointCloudPoint<float>> points = std::move( _cloud.pts );
		_cloud.pts = std::vector<PointCloudPoint<float>>();

		_setupIndex();

        for( int i = 0; i < _elements.size(); ++i )
        {
            auto& element = _elements[i];
            const auto position = element->position;
            
			points[i] = { position.x(), position.y(), position.z() };
        }

		_cloud.pts = points;

		if( _cloud.pts.size() > 0 )
			_index->addPoints( 0, _cloud.pts.size() - 1 );
    }
    
    PointCloud<PointCloudPoint<float> > _cloud;
    IndexAdaptor_t* _index = nullptr;

    std::vector<FElement*> _elements;
};