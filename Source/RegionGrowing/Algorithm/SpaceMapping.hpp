//
//  SpaceMapping.hpp
//  RegionGrowing
//
//  Created by Timothy Davison on 2015-09-10.
//  Copyright © 2015 EpicGames. All rights reserved.
//

#pragma once

#include "tcodsMeshInterface.h"
#include "Algorithm/tcods/HalfEdge.h"
#include "Utility.h"

#include <utility>

#include "Eigen/Geometry"

namespace rg
{
    typedef Eigen::Hyperplane<float, 3> Plane;
    
    
    class Mapping
    {
    public:
        struct Result
        {
            bool hit;
            Eigen::Vector3f position;
            Eigen::Vector3f normal;
            uint32_t face;
        };

        Mapping(tcodsMeshInterface * meshInterface_in,
                       Eigen::Vector3f const & surfaceOrigin_in,
                       Eigen::Vector3f const & exemplarOrigin_in)
        {
            meshInterface = meshInterface_in;
            surfaceOrigin = surfaceOrigin_in;
            exemplarOrigin = exemplarOrigin_in;
        }
        
        void setExemplarOrigin(Eigen::Vector3f const & exemplarOrigin_in)
        {
            exemplarOrigin = exemplarOrigin_in;
        }
        
        virtual auto toExemplar(Eigen::Vector3f surfacePosition) -> Eigen::Vector3f = 0;
        virtual auto toExemplarVector(Eigen::Vector3f surfaceVector) -> Eigen::Vector3f = 0;

        virtual auto toSurface(Eigen::Vector3f exemplarPosition) -> Result = 0;
        
		// by default, do we do distance comparisons in exemplar space?
		virtual bool compareInExemplarSpace() { return true;  }

    protected:
        tcodsMeshInterface * meshInterface;
        
        Eigen::Vector3f exemplarOrigin;
        Eigen::Vector3f surfaceOrigin;
    };
    
    class SpaceMapping : public Mapping
    {
    public:
        SpaceMapping(tcodsMeshInterface * meshInterface_in,
                       Eigen::Vector3f const & surfaceOrigin_in,
                       Eigen::Vector3f const & exemplarOrigin_in)
        : Mapping(meshInterface_in, surfaceOrigin_in, exemplarOrigin_in)
        {
        }
        
        virtual auto toExemplar(Eigen::Vector3f outputPosition) -> Eigen::Vector3f
        {
            return outputPosition - surfaceOrigin + exemplarOrigin;
        }
        
        virtual auto toExemplarVector(Eigen::Vector3f surfaceVector) -> Eigen::Vector3f
        {
            return surfaceVector;
        }
        
        virtual auto toSurface(Eigen::Vector3f exemplarPosition) -> Result
        {
            Result result;
            result.hit = true;
            result.position = exemplarPosition - exemplarOrigin + surfaceOrigin;
            result.normal = {0.0f, 0.0f, 0.0f};
            result.face = 0;
            
            return result;
        }
    };
    
    class SurfaceMapping : public Mapping
    {
    public:
        SurfaceMapping(tcodsMeshInterface * meshInterface_in,
                     Eigen::Vector3f const & surfaceOrigin_in,
                     Eigen::Vector3f const & exemplarOrigin_in)
        : Mapping(meshInterface_in, surfaceOrigin_in, exemplarOrigin_in)
        {
            meshInterface = meshInterface_in;
            
            auto nearest = meshInterface->nearestPointOnMesh(unreal(surfaceOrigin_in));
            
            auto face = meshInterface->mesh.faces[nearest.faceIndex];
            
            auto normal = eigen(face.normal());
            
            tangentPlane = Plane(normal, surfaceOrigin);
            
            using namespace tcods;
            
            double alpha = face.alpha;
            
            Vector e1, e2; face.frame(e1, e2);
            Vector n = e2 ^ e1;
            Vector u = e1 * cos(alpha) + e2 * sin(alpha);
            
            // the vector orthogonal to u in the plane of e1, e2
            const double pi_2 = Math::pid() * .5;
            FVector xAxis = unreal(e1 * cos(alpha + pi_2) + e2 * sin(alpha + pi_2));
            FVector yAxis = unreal(face.normal());
            FVector zAxis = FVector::CrossProduct(yAxis, xAxis);
            
            toSurfaceRotation = FTransform(xAxis, zAxis, yAxis, FVector::ZeroVector);
            toExemplarRotation = toSurfaceRotation.Inverse();
        }
        
        SurfaceMapping(tcodsMeshInterface * meshInterface_in,
                     Eigen::Vector3f const & surfaceOrigin_in,
                     uint32_t surfaceFaceIndex_in,
                     Eigen::Vector3f const & exemplarOrigin_in)
        : Mapping(meshInterface_in, surfaceOrigin_in, exemplarOrigin_in)
        {
            auto face = meshInterface->mesh.faces[surfaceFaceIndex_in];
            
            auto normal = eigen(face.normal());
            
            tangentPlane = Plane(normal, surfaceOrigin);
            
            using namespace tcods;
            
            double alpha = face.alpha;
            
            Vector e1, e2; face.frame(e1, e2);
            Vector n = e2 ^ e1;
            Vector u = e1 * cos(alpha) + e2 * sin(alpha);
            
            // the vector orthogonal to u in the plane of e1, e2
            const double pi_2 = Math::pid() * .5;
            FVector xAxis = unreal(e1 * cos(alpha + pi_2) + e2 * sin(alpha + pi_2));
            FVector yAxis = unreal(face.normal());
            FVector zAxis = FVector::CrossProduct(yAxis, xAxis);
            
            toSurfaceRotation = FTransform(xAxis, zAxis, yAxis, FVector::ZeroVector);
            toExemplarRotation = toSurfaceRotation.Inverse();
        }
        
		virtual ~SurfaceMapping()
		{
		}
        
        virtual auto toExemplar(Eigen::Vector3f surfacePosition) -> Eigen::Vector3f
        {
            auto planePoint = tangentPlane.projection(surfacePosition);
            
            auto exemplarVector = eigen(toExemplarRotation.TransformVectorNoScale(unreal(planePoint - surfaceOrigin)));
            
            auto exemplarPoint = exemplarOrigin + exemplarVector;
            
            return exemplarPoint;
        }
        
        virtual auto toExemplarVector(Eigen::Vector3f surfaceVector) -> Eigen::Vector3f
        {
            auto exemplarVector = eigen(toExemplarRotation.TransformVectorNoScale(unreal(surfaceVector)));
            exemplarVector.z() = 0.0f;
            
            return exemplarVector;
        }
        
        virtual auto toSurface(Eigen::Vector3f exemplarPosition) -> Result
        {
            using namespace Eigen;
            using namespace tcods;
            
            Vector3f neighbourVector = exemplarPosition - exemplarOrigin;
            
            neighbourVector = eigen(toSurfaceRotation.TransformVectorNoScale(unreal(neighbourVector)));
            
            FVector normal = unreal(tangentPlane.normal());
            FVector origin = unreal(surfaceOrigin + neighbourVector) + normal * 16.0f;
            
            auto hit = meshInterface->getIntersectionAndFace(origin, -normal);
            
            uint32_t faceIndex = std::get<2>(hit);
            
            auto face = meshInterface->mesh.faces[faceIndex];
            
            Vector e1, e2;
            face.frame(e1, e2);
            Vector n = e2 ^ e1;
            
            Result result;
            result.hit = std::get<0>(hit);
            result.normal = eigen(n);
            result.position = eigen(std::get<1>(hit)) + result.normal * exemplarPosition.z();
            result.face = faceIndex;
            
            return result;
        }
        
    private:
        Plane tangentPlane;
        
        FTransform toSurfaceRotation;
        FTransform toExemplarRotation;
    };
    
    class SurfaceWalk : public Mapping
    {
    public:
        SurfaceWalk(tcodsMeshInterface * meshInterface_in,
                     Eigen::Vector3f const & surfaceOrigin_in,
                     Eigen::Vector3f const & exemplarOrigin_in)
        : Mapping(meshInterface_in, surfaceOrigin_in, exemplarOrigin_in)
        {
            tcodsMeshInterface::NearestPoint nearest = meshInterface->nearestPointOnMesh(unreal( surfaceOrigin_in ));			
		
			surfaceOrigin = eigen(nearest.point);
			_face = meshInterface->mesh.faces.begin() + nearest.faceIndex;
        }
        
        /**
         If one already has a face index, we can avoid an expensive nearestPointOnMesh lookup
         that we would get with the other constructor.
         */
        SurfaceWalk(tcodsMeshInterface * meshInterface_in,
                    Eigen::Vector3f const & surfaceOrigin_in,
                    uint32_t startFace_in,
                    Eigen::Vector3f const & exemplarOrigin_in)
        : Mapping(meshInterface_in, surfaceOrigin_in, exemplarOrigin_in)
        {
			_face = meshInterface->mesh.faces.begin() + startFace_in;
        }

		virtual auto toExemplar( Eigen::Vector3f surfacePosition )->Eigen::Vector3f
		{
			return Eigen::Vector3f::Zero();
		}

		virtual auto toExemplarVector( Eigen::Vector3f surfaceVector )->Eigen::Vector3f
		{
			return Eigen::Vector3f::Zero();
		}

		virtual auto toSurface( Eigen::Vector3f exemplarPosition )->Result
		{
			using namespace Eigen;

			Result result;

			Vector3f direction = exemplarPosition - exemplarOrigin; 

			float distance = direction.norm();
			if(distance != 0.0f)
				direction /= distance;

			FQuat r = meshInterface->rotationAndNormalAtIndex( _face->index ).first;
			FVector direction_u = r.RotateVector( unreal(direction) );
			
            // compute the tangent direction on the start face
			double initialAngle = atan2( direction.y(), direction.x() );

            // integralWalk's start point is in the local frame of the starting face
            tcods::Vector startingPoint = _face->toLocal( to_tcods( surfaceOrigin ) );

			bool faceContainsStartingPoint = _face->contains( to_tcods( surfaceOrigin ) );

			//if(!faceContainsStartingPoint)
			//{
			//	UE_LOG( LogTemp, Warning, TEXT( "_face doesn't contain point" ) );
			//}


			auto finalFace = meshInterface->mesh.integralWalk( _face, startingPoint, initialAngle, 10, distance );

            if(finalFace.first == meshInterface->mesh.faces.end())
            {
                result.hit = false;

                result.normal = Vector3f( 0.0f, 0.0f, 1.0f );
                result.position = surfaceOrigin;
                result.face = _face->index;
            }
            else
            {
                tcods::Vector globalPosition = finalFace.first->toGlobal( finalFace.second );

                result.hit = true;
                result.normal = Vector3f( 0.0f, 0.0f, 1.0f );
                result.position = eigen( globalPosition );
                result.face = finalFace.first->index;
            }

			return result;
		}

		virtual bool compareInExemplarSpace()
		{
			return false;
		}

	private:


        
    private:
        
        
		tcods::FaceIter _face;
    };
}

