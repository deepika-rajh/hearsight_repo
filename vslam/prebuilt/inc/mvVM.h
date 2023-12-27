/*****************************************************************************
@copyright
Copyright (c) 2021-2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef MVVM_H
#define MVVM_H

//==============================================================================
// Defines
//==============================================================================


//==============================================================================
// Includes
//==============================================================================

#include <mv.h>
#include <string.h>

//==============================================================================
// Declarations
//==============================================================================

#ifdef __cplusplus
extern "C"
{
#endif

   /************************************************************************//**
   @detailed
      Voxel Mapping (VM)
   ****************************************************************************/
   typedef struct mvVM mvVM;


   /************************************************************************//**
   @detailed
      Configuration structure for integration functions.
   @param noise
      Noise (in meters) associated with range measurements.  The noise model
      assumes the same noise everywhere and uses a constant ramp length.  This
      noise model should fit ToF sensors well, but less so structured light
      sensors although it would still give reasonable values.  A value of 0.1
      is probably a good place to start but the optimal value is best found
      through tuning for your particular use case.
   @param filterWeight
      Filter delay, essentially the length of the moving average filter.  It is
      a count so is dimensionless.  The filterWeight acts like a cap on the
      weighting of previous values.  Increasing the weight to a large number
      would be equivalent to computing an average of all measurements ever
      made for a voxel, while capping the weight kind of creates a moving
      average filter.  That implicitly allows voxels to decay with new
      measurements.  A value of 30 might be a good starting point but this will
      require use case specific tuning.
   @param updateMode
      What parts of the voxel grid are updated with new range data.
      \n\b SURFACE:  only updates voxels in the range of the noise ramp which
      works well for static scenes and low noise sensors (e.g., not DFS).
      \n\b VISIBLE:  updates all voxels in the noise ramp and the empty space
      before it which is effectively the whole frustum visible to the depth
      sensor, limited by the measured depth values.  This costs more work to
      be done but can better filter out noise and so is recommended for robot
      planning.
      \n\b EXISTING_VISIBLE:  tradeoff between SURFACE and VISABLE.  It filters
      over measurements in the frustum if they already exist.  So, free space
      is not explicitly modeled but noisy measurements can be removed again.
      This was specifically created to deal with the noise from DFS while
      costing less than VISIBLE.
   ****************************************************************************/
   struct mvVM_IntegrationConfiguration
   {
      float32_t noise;
      float32_t filterWeight;

	  float32_t hitThreshold;
	  float32_t freeThreshold;

      enum
      {
         SURFACE,          ///< only update around measurements
         VISIBLE,          ///< update whole visible frustum
         EXISTING_VISIBLE  ///< update existing voxels in the visible frustum
      } updateMode;
   };

   /************************************************************************//**
	@detailed
	   Limintation information during mapping building.
	@param height
	   Discard the point with its height larger than this threshold to speed up
	   and save the memory.
	@param pixelStep
	   Step of image pixel during selecting the occupied blocks to speed up. This
	   value has some depends on the sensor's properties.
	****************************************************************************/
   struct mvVM_Limitation
   {
	   float32_t height;
	   unsigned int pixelStep;
   };

   /************************************************************************//**
   @detailed
      Return data for collision checking and distance computation functions.
   @param point
      3D sample point location in meters that collided, this is only valid for
      collision types MV_COLLISION_YES and MV_COLLISION_UNKNOWN.
   @param type
      Return value of the collision detection.
   ****************************************************************************/
   struct mvVM_CollisionInfo
   {
      float32_t point[3];
      MV_COLLISION type;
   };


   /************************************************************************//**
   @detailed
      Initialize a Voxel Map object.
   @param sampleDistance
      Distances in meters between samples along X, Y, and Z axis.
   @return
      On success pointer to VM object, NULL pointer on failure.
   ****************************************************************************/
   MV_API mvVM* mvVM_Initialize( const float32_t sampleDistance[3] );


   /************************************************************************//**
   @detailed
      Deinitialize VM object
   @param map
      VM object.
   ****************************************************************************/
   MV_API void mvVM_Deinitialize( mvVM* map );


   /************************************************************************//**
   @detailed
      Get the sample distances in meters for the map.
   @param map
      VM object.
   @param sampleDistance
      Contains the sample distances in meters along X, Y and Z axis.
   ****************************************************************************/
   MV_API void mvVM_GetSampleDistance( mvVM* map, float32_t sampleDistance[3] );


   /************************************************************************//**
   @detailed
      Moves the origin of the map to a new origin.
   @param map
      VM object.
   @param origin
      New origin in meters and current world coordinates.  The value origin is
      changed to reflect the actual new origin which may differ from the
      provided one, due to quantization of the map samples.
   ****************************************************************************/
   MV_API void mvVM_MoveOriginTo( mvVM* map, float32_t origin[3] );


   /************************************************************************//**
   @detailed
      Clears the data in the map
   @param map
      VM object
   ****************************************************************************/
   MV_API void mvVM_Clear( mvVM* map );


   /************************************************************************//**
   @detailed
      Integrates a new depth map into the map.
      \n\b NOTE:  Updates the volume map by integrating the depth map.
   @param map
      VM object.
   @param data
      Pointer to the raw depth map image data.  The values are interpreted as
      depth measurements in the same units as the sample distances.
   @param camera
      Pointer to a camera calibration object.  By default, a linear camera model
      is assumed.  Non-linear distortion parameters are supported if set.
   @param registration
      Pointer to a pose object, storing the transformation from world
      coordinate system to camera coordinate system.
   @param noiseModel
      Single parameter noise model for the depth map, here the "ramp" around
      the measured depth values.
   @param filterModel
      Single parameter filter model for the map, here the maximal weight of
      the running average filter.
   @param limitedDepth
      Configurate limitedDepth as true, will consider the depth measurement accuracy during integration
   ****************************************************************************/
   MV_API void mvVM_IntegrateDepthMap( mvVM* map, const float32_t* data,
                                const mvCameraConfiguration* camera,
                                const mvPose6DRT* registration,
                                const mvVM_IntegrationConfiguration* config,
								bool limitedDepth = false );
    
   /************************************************************************//**
	@detailed
		Integrates a new depth map into the map.
		\n\b NOTE:  Updates the volume map by integrating the depth map.
	@param map
		VM object.
	@param data
		Pointer to the raw depth map image data.  The values are interpreted as
		depth measurements in the same units as the sample distances.
	@param camera
		Pointer to a camera calibration object.  Non-linear distortion parameters
		are not supported, a linear camera model is assumed.
	@param registration
		Pointer to a pose object, storing the transformation from world
		coordinate system to camera coordinate system.
	@param noiseModel
		Single parameter noise model for the depth map, here the "ramp" around
		the measured depth values.
	@param filterModel
		Single parameter filter model for the map, here the maximal weight of
		the running average filter.
	@param maxHeight
		Discard the point with height larger than maxHeight
	****************************************************************************/
   MV_API void mvVM_IntegrateDepthMapLimitedHeight(mvVM* map, const float32_t* data,
	   const mvCameraConfiguration* camera,
	   const mvPose6DRT* registration,
	   const mvVM_IntegrationConfiguration* config,
	   const mvVM_Limitation limit);

   /************************************************************************//**
   @detailed
      Integrates a new depth map into the map.
      \n\b NOTE:  Updates the volume map by integrating the depth map.
   @param map
      VM object.
   @param data
      Pointer to the raw depth map image data.  The values are interpreted as
      depth measurements in the same units as the sample distances.
   @param camera
      Pointer to a camera calibration object.  By default, a linear camera model
      is assumed.  Non-linear distortion parameters are supported if set.
   @param registration
      Pointer to a pose object, storing the transformation from world
      coordinate system to camera coordinate system.
   @param config
      Pointer to integration configuration data.
   ****************************************************************************/
   MV_API void mvVM_IntegrateDepthMapUInt16( mvVM* map, const uint16_t* data,
                                const mvCameraConfiguration* camera,
                                const mvPose6DRT* registration,
                                const mvVM_IntegrationConfiguration* config,
								bool limitedDepth = false );

   /************************************************************************//**
	@detailed
		Integrates a new depth map into the map.
		\n\b NOTE:  Updates the volume map by integrating the depth map.
	@param map
		VM object.
	@param data
		Pointer to the raw depth map image data.  The values are interpreted as
		depth measurements in the same units as the sample distances.
	@param camera
		Pointer to a camera calibration object.  Non-linear distortion parameters
		are not supported, a linear camera model is assumed.
	@param registration
		Pointer to a pose object, storing the transformation from world
		coordinate system to camera coordinate system.
	@param config
		Pointer to integration configuration data.
	@Param maxHeight
		Discard the point with height larger than maxHeight
	****************************************************************************/
   MV_API void mvVM_IntegrateDepthMapUInt16LimitedHeight(mvVM* map, const uint16_t* data,
	   const mvCameraConfiguration* camera,
	   const mvPose6DRT* registration,
	   const mvVM_IntegrationConfiguration* config,
	   const mvVM_Limitation limit);




   /* the following functions only check for a collision between a query
      geometry and any occupied parts of the map
      they return true if a collision occurred, false otherwise */

   /************************************************************************//**
   @detailed
      Checks if a point in space is occupied.
   @param map
      VM object.
   @param A
      Location of the point in 3D space in the world coordinate frame.
   @param threshold
      Map threshold value to tread a sample in the map as occupied.
   @param info
      Optional structure to return the sample point that collided and more
      information.  If info == NULL, then it is ignored.
   @return
      MV_COLLISION value describing the result as no collision, collision,
      or unknown.
   ****************************************************************************/
   MV_API MV_COLLISION mvVM_CheckCollisionWithPoint( const mvVM* map,
                                                     const float32_t A[3],
                                                     float32_t threshold,
                                                     mvVM_CollisionInfo* info );


   /************************************************************************//**
   @detailed
      Checks if an axis aligned box in space hits the map
   @param map
      VM object.
   @param lower
      Lower corner of the box.
   @param upper
      Upper corner of the box.
   @param threshold
      Map threshold value to tread a sample in the map as occupied.
   @param info
      Optional structure to return the sample point that collided and more
      information.  If info == NULL, then it is ignored.
   @return
      MV_COLLISION value describing the result as no collision, collision,
      or unknown.
   ****************************************************************************/
   MV_API MV_COLLISION mvVM_CheckCollisionWithBox( const mvVM* map,
                                                   const float32_t lower[3],
                                                   const float32_t upper[3],
                                                   float32_t threshold,
                                                   mvVM_CollisionInfo* info );


   /************************************************************************//**
   @detailed
      Checks if a line in space hits the map.
   @param map
      VM object.
   @param A
      Start point of the line.
   @param B
      End point of the line.
   @param threshold
      Map threshold value to tread a sample in the map as occupied.
   @param info
      Optional structure to return the sample point that collided and more
      information.  If info == NULL, then it is ignored.
   @return
      MV_COLLISION value describing the result as no collision, collision,
      or unknown.
   ****************************************************************************/
   MV_API MV_COLLISION mvVM_CheckCollisionWithLine( const mvVM* map,
                                                    const float32_t A[3],
                                                    const float32_t B[3],
                                                    float32_t threshold,
                                                   mvVM_CollisionInfo* info );



   /* the following functions provide information on the clearance between a
      query geometry and any occupied parts of the map */

   /************************************************************************//**
   @detailed
      Returns the closest hit point on the map and the distance to a given
      point.  The returned point does not need to be the unique solution, there
      might be more points with the same distance.
   @param map
      VM object.
   @param A
      Point in space.
   @param maximalDistance
      Maximal distance to search for.
   @param threshold
      Map threshold value to tread a sample in the map as occupied.
   @param distance
      Pointer to return the distance found.
   @param minimalPoint
      Map sample location that was found to be closest.
   ****************************************************************************/
   MV_API MV_COLLISION mvVM_GetMinimalDistanceToPoint( const mvVM* map,
                                               const float32_t A[3],
                                               float32_t maximalDistance,
                                               float32_t threshold,
                                               float32_t* distance,
                                               mvVM_CollisionInfo* info );


   /************************************************************************//**
   @detailed
      Returns the closest hit point on the map and the distance to a given axis
      aligned box.  The returned point does not need to be the unique solution,
      there might be more points with the same distance.
   @param map
      VM object.
   @param lower
      Lower corner of the box.
   @param upper
      Upper corner of the box.
   @param maximalDistance
      Maximal distance to search for.
   @param threshold
      Map threshold value to tread a sample in the map as occupied.
   @param distance
      Pointer to return the distance found.
   @param minimalPoint
      Map sample location that was found to be closest.
   ****************************************************************************/
   MV_API MV_COLLISION mvVM_GetMinimalDistanceToBox( const mvVM* map,
                                                const float32_t lower[3],
                                                const float32_t upper[3],
                                                float32_t maximalDistance,
                                                float32_t threshold,
                                                float32_t* distance,
                                                mvVM_CollisionInfo* info );


   /* The following functions trim a volume map by removing all data outside of
      the given clipping geometry */

   /************************************************************************//**
   @detailed
      Clips the map against a given axis aligned box.  All data within the map
      outside of the box is deleted. Some fringe around the box can remain.
   @param map
      VM object.
   @param lower
      Lower corner of the box.
   @param upper
      Upper corner of the box.
   ****************************************************************************/
   MV_API void mvVM_ClipAgainstBox( mvVM* map, const float32_t lower[3],
                                    const float32_t upper[3] );


   /************************************************************************//**
   @detailed
      Clips the map against a given sphere.  All data within the map outside of
      the sphere is deleted. Some fringe around the sphere can remain.
   @param map
      VM object.
   @param center
      Center of the sphere.
   @param radius
      Radius of the sphere.
   ****************************************************************************/
   MV_API void mvVM_ClipAgainstSphere( mvVM* map, const float32_t center[3],
                                       float32_t radius );


   /************************************************************************//**
   @detailed
      Sets samples in the volume covered by a box to free.
   @param map
      VM object.
   @param lower
      Lower corner of the box.
   @param upper
      Upper corner of the box.
   ****************************************************************************/
   MV_API void mvVM_SetBoxFree( mvVM* map, const float32_t lower[3],
                                const float32_t upper[3] );


   /************************************************************************//**
   @detailed
      Sets samples in the volume covered by a box to be occupied.
   @param map
      VM object.
   @param lower
      Lower corner of the box.
   @param upper
      Upper corner of the box.
   ****************************************************************************/
   MV_API void mvVM_SetBoxOccupied( mvVM* map, const float32_t lower[3],
                                    const float32_t upper[3] );


   /* All extraction functions return unknown amounts of data.
      They implement the following API to do this:
      - points, vertices, indices point to arrays allocated by the caller to
        receive the output data
      - The size of the arrays is passed in through numberPoints,
        numberVertices and numberIndices
      - If enough space was available in the arrays,
         - the data will be copied into the arrays,
         - and the numberXXXX variables will be updated to hold the number of
           valid data items
      - else (not enough space available)
         - the numberXXXX variables will be set to the required minimum number
         of data items
      - To understand, if data was copied into the arrays, check, if the number
        of data items after the call is less or equal to the sizes passed in */

   /************************************************************************//**
   @detailed
      Extracts the occupied sample locations in the volume grid that have
      non-empty values. This can be used to create a representation of the
      occupied blocks - not the estimated surface.
   @param map
      VM object.
   @param points
      Pointer to a buffer of floats in {x0,y0,z0}, {x1,y1,z1}, ... format.
      Use NULL if wanting numberPoints first:
      size_t numberVertices = 0;
      mvVM_ExtractSamplePoints( map, 0, NULL, &numberVertices );
   @param numberPoints
      Pointer to size value of the number of points written to the buffer.
   ****************************************************************************/
   MV_API void mvVM_ExtractSamplePoints( const mvVM* map,
                                         float32_t threshold,
                                         float32_t* points,
                                         size_t* numberPoints );


   /************************************************************************//**
   @detailed
      Extracts the occupied sample locations in the volume grid that have
      non-empty values. This can be used to create a representation of the
      occupied blocks - not the estimated surface.
   @param map
      VM object.
   @param points
      Pointer to a buffer of floats in {x0,y0,z0}, {x1,y1,z1}, ... format.
      Use NULL if wanting numberPoints first:
      size_t numberVertices = 0;
      mvVM_ExtractSamplePoints( map, 0, NULL, &numberVertices );
   @param numberPoints
      Pointer to size value of the number of points written to the buffer.
   ****************************************************************************/
   MV_API void mvVM_ExtractMapPoints(const mvVM* map,
	   const float32_t thresholdmin,
	   const float32_t thresholdmax,
	   float32_t* points,
	   size_t* numberPoints);


   /************************************************************************//**
   @detailed
      Extracts the vertex locations on the surface.
   @param map
      VM object.
   @param vertices
      Pointer to a buffer of floats in {x0,y0,z0}, {x1,y1,z1}, ... format.
      Use NULL if wanting numberPoints first:
      size_t numberVertices = 0;
      mvVM_ExtractSurfacePoints( map, 0, NULL, &numberVertices );
   @param numberVertices
      Pointer to size value of the number of points written to the buffer.
   ****************************************************************************/
   MV_API void mvVM_ExtractSurfacePoints( const mvVM* map,
                                          float32_t threshold,
                                          float32_t* vertices,
                                          size_t* numberVertices );


   /************************************************************************//**
   @detailed
      Extracts a surface mesh.
   @param map
      VM object.
   @param vertices
      Pointer to a buffer of floats in {x0,y0,z0}, {x1,y1,z1}, ... format.
   @param numberVertices
      Pointer to size value of the number of points written to the buffer.
   @param indices
      Pointer to a buffer of triangle vertex indices of the mesh.
   @param numberIndices
      Pointer to size value of the number of indices written to the buffer.
   ****************************************************************************/
   MV_API void mvVM_ExtractSurfaceMesh( const mvVM* map,
                                        float32_t threshold,
                                        float32_t* vertices,
                                        size_t* numberVertices,
                                        uint32_t* indices,
                                        size_t* numberIndices );


#ifdef __cplusplus
}
#endif

#ifdef __cplusplus


namespace MV
{

/***************************************************************************//**
@detailed
   Reconstruction
@detailed
   Higher-level APIs as inline C++ classes for reconstruction.
*******************************************************************************/

class Reconstruction
{
public:

   // no copy
   Reconstruction( const Reconstruction& ) = delete;

   // no assign
   Reconstruction& operator=( const Reconstruction& ) = delete;

   Reconstruction( const float32_t sampleDistance[3] ) :
      map( mvVM_Initialize( sampleDistance ) )
   {
      // assert( map != NULL );
      configuration.noise = 0.1f;
      configuration.filterWeight = 30;
      configuration.updateMode = mvVM_IntegrationConfiguration::SURFACE;

      registration = mvPose6DRT{
          1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, 0 };
   }

   ~Reconstruction( void )
   {
      mvVM_Deinitialize( map );
   }

   void setCamera( const mvCameraConfiguration& cam )
   {
      camera = cam;
   }

   void setRegistration( const mvPose6DRT& pose )
   {
      registration = pose;
   }

   void setNoiseModel( float n )
   {
      configuration.noise = n;
   }

   void setFilterModel( float f )
   {
      configuration.filterWeight = f;
   }

   void integrate( const float32_t* data, const mvPose6DRT* pose )
   {
      // transform = registration * pose;
      mvPose6DRT transform;
      mvMultiplyPose6DRT( &registration, pose, &transform );
      mvVM_IntegrateDepthMap( map, data, &camera, &transform, &configuration );
   }

   void clear( void )
   {
      mvVM_Clear( map );
   }

   void extractSurfacePoints( float32_t* points, size_t* size ) const
   {
      mvVM_ExtractSurfacePoints( map, 0, points, size );
   }

   void extractSurfaceMesh( float32_t* vertices, size_t* vertexSize,
                            uint32_t* indices, size_t* indexSize ) const
   {
      mvVM_ExtractSurfaceMesh( map, 0, vertices, vertexSize, indices, indexSize );
   }

   mvVM* getMap( void )
   {
      return map;
   }

   const mvVM* getMap( void ) const
   {
      return map;
   }


protected:
   mvVM* map;
   mvCameraConfiguration camera;
   mvPose6DRT registration;
   mvVM_IntegrationConfiguration configuration;
};



/***************************************************************************//**
@detailed
   OccupancyMap
@detailed
   Higher-level APIs as inline C++ classes for occupancy testing.
*******************************************************************************/

class OccupancyMap
{
public:

   // no copy
   OccupancyMap( const OccupancyMap& ) = delete;

   // no assign
   OccupancyMap& operator=( const OccupancyMap& ) = delete;

   OccupancyMap( const float32_t sampleDistance[3] ) :
      map( mvVM_Initialize( sampleDistance ) ),
      extend( 5.f )
   {
      // assert( map != NULL );
      configuration.noise = 0.1f;
      configuration.filterWeight = 30;
      configuration.updateMode = mvVM_IntegrationConfiguration::VISIBLE;

      registration = mvPose6DRT{
          1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, 0 };

      newOrigin[0] = 0.f;
      newOrigin[1] = 0.f;
      newOrigin[2] = 0.f;

      offset = registration;
   }

   ~OccupancyMap( void )
   {
      mvVM_Deinitialize( map );
   }

   void setCamera( const mvCameraConfiguration& cam )
   {
      camera = cam;
   }

   void setRegistration( const mvPose6DRT* pose )
   {
      registration = *pose;
   }

   void setNoiseModel( float n )
   {
      configuration.noise = n;
   }

   void setFilterModel( float f )
   {
      configuration.filterWeight = f;
   }

   void setRadius( float r )
   {
      extend = r;
   }

   MV_API void integrate( const float32_t* data );

   void move( const mvPose6DRT* motion )
   {
      {
         const mvPose6DRT oldOffset = offset;
         mvMultiplyPose6DRT( motion, &oldOffset, &offset );
      }

      mvPose6DRT inverseOffset = offset;
      mvInvertPose6DRT( &inverseOffset );
      newOrigin[0] = inverseOffset.matrix[0][3];
      newOrigin[1] = inverseOffset.matrix[1][3];
      newOrigin[2] = inverseOffset.matrix[2][3];
      mvVM_MoveOriginTo( map, newOrigin );
      inverseOffset = mvPose6DRT{
          1, 0, 0, newOrigin[0],
          0, 1, 0, newOrigin[1],
          0, 0, 1, newOrigin[2] };

      {
         const mvPose6DRT oldOffset = offset;
         mvMultiplyPose6DRT( &oldOffset, &inverseOffset, &offset );
      }
   }

   void clear( void )
   {
      mvVM_Clear( map );
   }

   MV_COLLISION checkCollisionWithPoint( const float32_t A[3] ) const
   {
      return mvVM_CheckCollisionWithPoint( map, A, 0, NULL );
   }

   /// more of the collision methods

   void extractSamplePoints( float32_t* points, size_t* size ) const
   {
      const size_t before = *size;
      mvVM_ExtractSamplePoints( map, 0, points, size );
      if( *size <= before && points != nullptr )
      {
         mvPose6DRT inverseOffset = offset;
         //mvInvertPose6DRT( &inverseOffset );

         for( size_t i = 0; i < *size; i += 3 )
         {
            const float x = inverseOffset.matrix[0][0] * points[i] +
                            inverseOffset.matrix[0][1] * points[i + 1] +
                            inverseOffset.matrix[0][2] * points[i + 2] +
                            inverseOffset.matrix[0][3];
            const float y = inverseOffset.matrix[1][0] * points[i] +
                            inverseOffset.matrix[1][1] * points[i + 1] +
                            inverseOffset.matrix[1][2] * points[i + 2] +
                            inverseOffset.matrix[1][3];
            const float z = inverseOffset.matrix[2][0] * points[i] +
                            inverseOffset.matrix[2][1] * points[i + 1] +
                            inverseOffset.matrix[2][2] * points[i + 2] +
                            inverseOffset.matrix[2][3];
            points[i] = x;
            points[i + 1] = y;
            points[i + 2] = z;
         }
      }
   }

   void getOrigin( float32_t* origin )
   {
      memcpy( origin, newOrigin, 3 * sizeof( float32_t ) );
   }


protected:
   mvVM* map;
   mvCameraConfiguration camera;
   mvPose6DRT registration, offset;
   float32_t newOrigin[3];
   mvVM_IntegrationConfiguration configuration;

   float32_t extend;
};


} // namespace MV

#endif

#if defined _WIN32 && !defined MV_EXPORTS
#include "win/mvVM_DLLGlue.h"
#endif

#endif
