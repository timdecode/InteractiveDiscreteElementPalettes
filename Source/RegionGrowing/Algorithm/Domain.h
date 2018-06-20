//
//  Domain.h
//  RegionGrowing
//
//  Created by Timothy Davison on 2015-06-25.
//  Copyright (c) 2015 Timothy Davison. All rights reserved.
//

#pragma once

#include <vector>
#include <iostream>
#include <memory>
#include <numeric> 
#include <algorithm>
#include <iterator>

#include "Element.h"
#include "ElementKDTree.h"
#include "Entity.hpp"
#include "Utility.h"

//struct Domain
//{
//public:
//    struct FElementSemantics
//    {
//        enum { MaxElementsPerLeaf = 16 };
//        enum { MinInclusiveElementsPerNode = 7 };
//        enum { MaxNodeDepth = 12 };
//        
//        typedef TInlineAllocator<MaxElementsPerLeaf> ElementAllocator;
//        
//        FORCEINLINE static FBoxCenterAndExtent GetBoundingBox(const FElement * element)
//        {
//            return FBoxCenterAndExtent(unreal(element->position), FVector(element->radius * 2.0f));
//        }
//        
//        //FORCEINLINE static bool AreElementsEqual(const Element* a, const Element* b)
//        //{
//        //    return *a == *b;
//        //}
//
//        FORCEINLINE static void SetElementId(FElement* element, FOctreeElementId id)
//		{
//			element->octreeId = id;
//		}
//    };
//    
//    typedef TOctree<FElement*, FElementSemantics> ElementOctree;
//
//public:
//    
//
//
//	void init( Eigen::Vector3f origin_in, float extents_in )
//    {
//		origin = origin_in;
//		extents = extents_in;
//        
//        _octree = std::unique_ptr<ElementOctree>(new ElementOctree(unreal(origin), extents));
//    }
//    
//    // copies the elements into the domain
//    auto add(const FElement& element) -> FElement*
//    {
//        // we could manage our own memory block here
//        _elements.emplace_back(new FElement(element));
//        
//        FElement* result = _elements.back().get();
//        
//        _octree->AddElement(result);
//        
//        return result;
//    }
//    
//    void update( FElement* element )
//    {
//        // not implemented yet
//    }
//    
//    void erase( const std::vector<FElement*>& elements )
//    {
//        decltype(_elements) originals = std::move(_elements );
//        
//        _elements.clear();
//        
//        std::unordered_set<FElement*> set;
//        
//        for(auto e : elements)
//            set.insert( e );
//        
//        std::vector<FElement*> pointers;
//        
//        for(auto& e : originals)
//        {
//            auto found = set.find( e.get() );
//            if(found == set.end())
//            {
//                pointers.push_back( e.get() );
//                _elements.push_back( std::move( e ) );
//            }
//        }
//
//		// update the octree
//        for( FElement * e : elements )
//			_octree->RemoveElement( e->octreeId );
//    }
//    
//    void setElements(const std::vector<FElement>& elements)
//    {
//        clear();
//        
//        std::vector<FElement*> pointers;
//        
//        for( const FElement& original : elements )
//        {
//            std::unique_ptr<FElement> element(new FElement(original));
//            
//            pointers.push_back(element.get());
//            _elements.push_back(std::move(element));
//        }
//
//
//		for(FElement * e : pointers)
//			_octree->AddElement( e );
//    }
//    
//    auto emplaceEntityBack() -> Entity&
//    {
//        _entities.emplace_back();
//        
//        return _entities.back();
//    }
//    
//    void setEntities(const std::vector<Entity>& entities)
//    {
//        _entities.clear();
//        std::copy(entities.begin(), entities.end(), std::back_inserter(_entities));
//    }
//    
//    size_t size()
//    {
//        return _elements.size();
//    }
//    
//    size_t entitySize()
//    {
//        return _entities.size();
//    }
//    
//    FElement* elementAt(const unsigned int index)
//    {
//        return _elements[index].get();
//    }
//    
//    Entity& entityAt(const unsigned int index)
//    {
//        return _entities[index];
//    }
//    
//    std::vector<Entity>& entities()
//    {
//        return _entities;
//    }
//    
//    void clear()
//    {
//        _elements.clear();
//        _entities.clear();
//
//        _octree = std::unique_ptr<ElementOctree>(new ElementOctree(unreal(origin), extents));
//	}
//    
//    // queries
//    ElementKDTree::NearestResult nearest(const Eigen::Vector3f& point, float radius = 1.0f)
//    {
//		std::vector<float> distances;
//		std::vector<FElement*> result = nearestInRadius( point, radius, distances );
//
//		if(result.size())
//			return{ result[0], distances[0] };
//		else
//			return{ nullptr, 0.0f };
//    }
//    
//   
//    std::vector<FElement*> nearestInRadius(const Eigen::Vector3f& point, float radius, std::vector<float>& distances_out, int count = -1, bool sort = true, FElement * toIgnore = nullptr)
//    {
//		FBoxCenterAndExtent bounds( unreal(point), FVector( radius * 2.0f ) );
//
//		ElementOctree::TConstElementBoxIterator<> iterator( *_octree.get(), bounds );
//
//		std::vector<FElement*> found;
//
//		while(iterator.HasPendingElements())
//		{
//			FElement * element = iterator.GetCurrentElement();
//
//			float distSqrd = (element->position - point).squaredNorm();
//
//			if(distSqrd < radius * radius && element != toIgnore)
//			{
//				distances_out.push_back( distSqrd );
//				found.push_back( element );
//			}
//
//			iterator.Advance();
//		}
//
//		if(!sort)
//			return found;
//
//		std::vector<size_t> indices( distances_out.size() );
//		std::iota( indices.begin(), indices.end(), 0 );
//
//		std::sort( indices.begin(), indices.end(), [&]( size_t a, size_t b )
//		{
//			return distances_out[a] < distances_out[b];
//		} );
//
//
//		size_t stopAt = count < 0 ? indices.size() : std::min(size_t(count), indices.size());
//
//		std::vector<FElement*> result;
//		result.reserve( found.size() );
//		for(int j = 0; j < stopAt; j++)
//			result.push_back( found[indices[j]] );
//
//        return result;
//    }
//    
//    std::vector<FElement*> nearestInRadius(const Eigen::Vector3f& point, float radius, int count = -1, bool sort = true)
//    {
//		std::vector<float> distances;
//
//		return nearestInRadius( point, radius, distances, count, sort );
//    }
//    
//    std::vector<FElement*> neighbours(FElement * element, float radius, int count = -1, bool sort = true, std::vector<float>* distances_out = nullptr)
//    {
//		if(distances_out)
//			return nearestInRadius( element->position, radius, *distances_out, count, sort, element );
//		else
//		{
//			std::vector<float> distances;
//
//			return nearestInRadius( element->position, radius, distances, count, sort, element );
//		}
//    }
//    
//    // range for compatability
//    // let's switch to this idea:
//    // http://stackoverflow.com/a/352162
//    auto begin() -> decltype( std::vector<std::unique_ptr<FElement>>().begin() )
//    {
//        return _elements.begin();
//    }
//    
//    auto end() -> decltype( std::vector<std::unique_ptr<FElement>>().end() )
//    {
//        return _elements.end();
//    }
//    
//    void rebalance()
//    {
//    }
//    
//private:
//    
//    
//    
//private:
//    std::vector<std::unique_ptr<FElement>> _elements;
//    std::vector<Entity> _entities;
//    
//	Eigen::Vector3f origin;
//	float extents;
//
//    
//    std::unique_ptr<ElementOctree> _octree;
//};

 struct Domain
 {
 public:

     // copies the elements into the domain
     auto add(const FElement& element) -> FElement*
     {
         // we could manage our own memory block here
         _elements.emplace_back(new FElement(element));
        
         FElement* result = _elements.back().get();
        
         _kdTree.add(result);

         return result;
     }

 	void update( FElement* element )
 	{
 		// not implemented yet
 	}

     void erase( const std::vector<FElement*>& elements )
     {
         decltype(_elements) originals = std::move(_elements );

         _elements.clear();

         std::unordered_set<FElement*> set;

         for(auto e : elements)
             set.insert( e );

         std::vector<FElement*> pointers;

         for(auto& e : originals)
         {
             auto found = set.find( e.get() );
             if(found == set.end())
             {
                 pointers.push_back( e.get() );
                 _elements.push_back( std::move( e ) );
             }
         }

         _kdTree.setElements( pointers );
     }
    
     void setElements(const std::vector<FElement>& elements, bool createEntities = false)
     {
         clear();

         std::vector<FElement*> pointers;
        
		 // load the elements
         for( const FElement& original : elements )
         {
             std::unique_ptr<FElement> element(new FElement(original));
            
             pointers.push_back(element.get());
             _elements.push_back(std::move(element));
         }
        
		 // load the kd-tree
         _kdTree.setElements(pointers);

		 // create some entities
		if( createEntities )
		{
			_entities.clear();

			int elementIndex = 0;
			int entityIndex = 0;

			for(FElement * e : pointers)
			{
				e->entityIndex = entityIndex;

				Entity& entity = emplaceEntityBack();

				entity.entityID = 0;

				entity.elementIndices.push_back( elementIndex );

				entityIndex++;
				elementIndex++;
			}
		}
     }
    
     auto emplaceEntityBack() -> Entity&
     {
         _entities.emplace_back();
        
         return _entities.back();
     }
    
     void setEntities(const std::vector<Entity>& entities)
     {
         _entities.clear();
         std::copy(entities.begin(), entities.end(), std::back_inserter(_entities));
     }
    
     size_t size()
     {
         return _elements.size();
     }
    
     size_t entitySize()
     {
         return _entities.size();
     }
    
     FElement* elementAt(const unsigned int index)
     {
         return _elements[index].get();
     }
    
     Entity& entityAt(const unsigned int index)
     {
         return _entities[index];
     }
    
     std::vector<Entity>& entities()
     {
         return _entities;
     }
    
     void clear()
     {
         _elements.clear();
         _entities.clear();
         _kdTree.clear();
     }
     
     // queries
     ElementKDTree::NearestResult nearest(const Eigen::Vector3f& point)
     {
         return _kdTree.nearest(point);
     }
    
     // queries
     ElementKDTree::NearestResult nearest(const Eigen::Vector3f& point, float raidus)
     {
         auto result = _kdTree.nearest(point);

		 if(result.distanceSquared < raidus * raidus)
			 return result;
		 else
			 return{ nullptr, 0.0f };
     }
    
     std::vector<FElement*> nearestKNeighbours(const Eigen::Vector3f& point, size_t count, std::vector<float>& distances_out)
     {
         return _kdTree.nearestKNeighbours(point, count, distances_out);
     }
    
     std::vector<FElement*> nearestKNeighbours(const Eigen::Vector3f& point, size_t count)
     {
         return _kdTree.nearestKNeighbours(point, count);
     }
    
     std::vector<FElement*> nearestInRadius(const Eigen::Vector3f& point, float radius, std::vector<float>& distances_out, int count = -1, bool sort = true)
     {
         return _kdTree.nearestInRadius(point, radius, distances_out, count, sort);
     }
    
     std::vector<FElement*> nearestInRadius(const Eigen::Vector3f& point, float radius, int count = -1, bool sort = true)
     {
         return _kdTree.nearestInRadius(point, radius, count, sort);
     }

	 // Hacked, we should really add a nearestInBox implementation to the k-d tree. This would be relatively trivial given that it
	 // is a k-d tree.
	 std::vector<FElement*> nearestInBox( const Eigen::Vector3f& point, const Eigen::Vector3f& halfExtents, int count = -1, bool sort = true )
	 {
		 // c^2 = a^2 + b^2
		 // so, a = b: c^2 = 2a^2
		 // hence, we have our radius
		 float maxHalfExtents = std::max( halfExtents( 0 ), std::max( halfExtents( 1 ), halfExtents( 2 ) ) );
		 float radius = sqrt( 2 ) * maxHalfExtents;

		 Eigen::AlignedBox3f aabb = Eigen::AlignedBox3f( point - halfExtents, point + halfExtents );

		 auto inRadius = _kdTree.nearestInRadius( point, radius, count, sort );

		 std::vector<FElement*> inBox;

		 for(FElement * e : inRadius)
		 {
			 if( aabb.contains(e->position) )
				 inBox.emplace_back( e );
		 }

		 return inBox;
	 }
    
     std::vector<FElement*> neighbours(FElement * element, float radius, int count = -1, bool sort = true, std::vector<float>* distances_out = nullptr, bool surfaceVolumeInteraction = true)
     {
         return _kdTree.neighbours(element, radius, count, sort, distances_out, surfaceVolumeInteraction );
     }
    
     // range for compatability
     // let's switch to this idea:
     // http://stackoverflow.com/a/352162
     auto begin() -> decltype( std::vector<std::unique_ptr<FElement>>().begin() )
     {
         return _elements.begin();
     }
    
     auto end() -> decltype( std::vector<std::unique_ptr<FElement>>().end() )
     {
         return _elements.end();
     }

	 std::vector<FElement*> elements()
	 {
		 std::vector<FElement*> elements;

		 for(auto& e : _elements)
			 elements.push_back( e.get() );

		 return elements;
	 }
    
     void rebalance()
     {
         _kdTree.rebalance();
     }

	 Eigen::AlignedBox3f computeAABB()
	 {
		 if( _elements.size() == 0 )
			 return Eigen::AlignedBox3f( Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero() );

		 Eigen::AlignedBox3f b( _elements[0]->position, _elements[0]->position );

		 for(int i = 1; i < _elements.size(); ++i)
		 {
			 b.extend( _elements[i]->position );
		 }

		 return b;
	 }
        
 protected:
     std::vector<std::unique_ptr<FElement>> _elements;
     std::vector<Entity> _entities;
    
     ElementKDTree _kdTree;
 };
