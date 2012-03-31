/*
* CinderBullet originally created by Peter Holzkorn on 2/16/10
* 
* Copyright (c) 2012, Ban the Rewind
* All rights reserved.
* 
* Redistribution and use in source and binary forms, with or 
* without modification, are permitted provided that the following 
* conditions are met:
* 
* Redistributions of source code must retain the above copyright 
* notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright 
* notice, this list of conditions and the following disclaimer in 
* the documentation and/or other materials provided with the 
* distribution.
* 
* Neither the name of the Ban the Rewind nor the names of its 
* contributors may be used to endorse or promote products 
* derived from this software without specific prior written 
* permission.
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
* ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
* 
*/

#pragma once

// Cinder includes
#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletSoftBody/btSoftBody.h"
#include "cinder/app/AppBasic.h"
#include "cinder/AxisAlignedBox.h"
#include "cinder/CinderMath.h"
#include "cinder/gl/Vbo.h"
#include "cinder/Matrix.h"
#include "cinder/Sphere.h"
#include "cinder/TriMesh.h"
#include "TetraCube.h"

namespace bullet 
{

	// Forward declarations
	class CollisionObjectBase;
	class RigidBox;
	class RigidCylinder;
	class RigidHull;
	class RigidMesh;
	class RigidObject;
	class RigidSphere;
	class RigidTerrain;
	class SoftObject;

	class Utilities
	{

	private:

		// Get mass for a collision shape
		static float						getMass( const btCollisionShape* shape );

		// Convert Bullet matrix to Cinder matrix for rigid bodies
		static ci::Matrix44f				getWorldTransform( const btRigidBody* body );

		// Convert Bullet matrix to Cinder matrix for soft bodies
		static ci::Matrix44f				getWorldTransform( const btSoftBody* body );

		// Convert Cinder vector to Bullet vector
		static btVector3					toBulletVector3( const ci::Vec3f &v );

		// Convert Bullet vector to Cinder vector
		static ci::Vec3f					fromBulletVector3( const btVector3 &v );

		// Convert Cinder quaternion to Bullet quaternion
		static btQuaternion					toBulletQuaternion( const ci::Quatf &q );

		// Convert Bullet quaternion to Cinder quaternion
		static ci::Quatf					fromBulletQuaternion( const btQuaternion &q );

		// Create terrain from color channel
		static btHeightfieldTerrainShape*	createHeightfieldTerrainShape( const ci::Surface32f &heightField, int32_t stickWidth, int32_t stickLength, 
			float heightScale, float minHeight, float maxHeight, int32_t upAxis, const ci::Vec3f &scale );

		// Creates a concave Bullet mesh from a TriMesh
		static btBvhTriangleMeshShape*		createConcaveMeshShape( const ci::TriMesh &mesh, const ci::Vec3f &scale, float margin );

		// Creates a concave Bullet mesh from a list of vertices and indices
		static btBvhTriangleMeshShape*		createConcaveMeshShape( const std::vector<ci::Vec3f> &vertices, const std::vector<uint32_t> &indices, 
			const ci::Vec3f &scale, float margin );

		// Creates a convex Bullet hull from a TriMesh
		static btConvexHullShape*			createConvexHullShape( const ci::TriMesh &mesh, const ci::Vec3f &scale );

		// Creates a convex Bullet hull from a list of vertices
		static btConvexHullShape*			createConvexHullShape( const std::vector<ci::Vec3f> &vertices, const ci::Vec3f &scale );

		// Grant classes permission to private methods
		friend class						CollisionObjectBase;
		friend class						RigidBox;
		friend class						RigidCylinder;
		friend class						RigidHull;
		friend class						RigidMesh;
		friend class						RigidObject;
		friend class						RigidSphere;
		friend class						RigidTerrain;
		friend class						SoftObject;

	};

} 
