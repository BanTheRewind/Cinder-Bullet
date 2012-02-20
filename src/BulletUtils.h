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
#ifdef CINDER_MSW
	#include "ppl.h"
#endif
#include "TetraCube.h"


namespace bullet 
{

	/****** TRANSFORM ******/

	/*! Convert Bullet matrix to Cinder matrix for rigid bodies */
	ci::Matrix44f getWorldTransform(const btRigidBody * body);

	/*! Convert Bullet matrix to Cinder matrix for soft bodies */
	ci::Matrix44f getWorldTransform(const btSoftBody * body);

	/*! Convert Cinder vector to Bullet vector */
	btVector3 toBulletVector3(const ci::Vec3f & v);

	/*! Convert Bullet vector to Cinder vector */
	ci::Vec3f fromBulletVector3(const btVector3 & v);

	/*! Convert Cinder quaternion to Bullet quaternion */
	btQuaternion toBulletQuaternion(const ci::Quatf & q);

	/*! Convert Bullet quaternion to Cinder quaternion */
	ci::Quatf fromBulletQuaternion(const btQuaternion & q);

	/****** MESH ******/

	/*! Create terrain from color channel */
	btHeightfieldTerrainShape * createHeightfieldTerrainShape(const ci::Surface32f & heightField, int32_t stickWidth, int32_t stickLength, float heightScale, float minHeight, float maxHeight, int32_t upAxis, const ci::Vec3f & scale);

	/*! Creates a concave Bullet mesh from a TriMesh */
	btBvhTriangleMeshShape * createConcaveMeshShape(const ci::TriMesh & mesh, const ci::Vec3f & scale = ci::Vec3f(1.0f, 1.0f, 1.0f), float margin = 0.0f);

	/*! Creates a concave Bullet mesh from a list of vertices and indices */
	btBvhTriangleMeshShape * createConcaveMeshShape(const std::vector<ci::Vec3f> & vertices, const std::vector<uint32_t> & indices, const ci::Vec3f & scale = ci::Vec3f(1.0f, 1.0f, 1.0f), float margin = 0.0f);

	/*! Creates a convex Bullet hull from a TriMesh */
	btConvexHullShape * createConvexHullShape(const ci::TriMesh & mesh, const ci::Vec3f & scale = ci::Vec3f(1.0f, 1.0f, 1.0f));

	/*! Creates a convex Bullet hull from a list of vertices */
	btConvexHullShape * createConvexHullShape(const std::vector<ci::Vec3f> & vertices, const ci::Vec3f & scale = ci::Vec3f(1.0f, 1.0f, 1.0f));

	/****** RIGID BODIES ******/

	/*! Creates rigid body from collision shape */
	btRigidBody * create(btDynamicsWorld * world, btCollisionShape * shape, const ci::Vec3f & position = ci::Vec3f::zero(), const ci::Quatf & rotation = ci::Quatf());

	/*! Create rigid body from triangle mesh */
	btRigidBody * create(btDynamicsWorld * world, btBvhTriangleMeshShape * shape, const ci::Vec3f & position = ci::Vec3f::zero(), const ci::Quatf & rotation = ci::Quatf());

	/*! Creates rigid body from convex hull shape */
	btRigidBody * create(btDynamicsWorld * world, btConvexHullShape * shape, const ci::Vec3f & position = ci::Vec3f::zero(), const ci::Quatf & rotation = ci::Quatf());

	/*! Creates a rigid box */
	btRigidBody * create(btDynamicsWorld * world, const ci::Vec3f & size = ci::Vec3f(10.0f, 10.0f, 10.0f), const ci::Vec3f & position = ci::Vec3f::zero(), const ci::Quatf & rotation = ci::Quatf());

	/*! Creates a rigid cylinder */
	btRigidBody * create(btDynamicsWorld * world, float topRadius = 10.0f, float bottomRadius = 10.0f, float height = 20.0f, int32_t segments = 16, const ci::Vec3f & position = ci::Vec3f::zero(), const ci::Quatf & rotation = ci::Quatf());

	/*! Creates a rigid sphere */
	btRigidBody * create(btDynamicsWorld * world, float radius = 10.0f, const ci::Vec3f & position = ci::Vec3f::zero(), const ci::Quatf & rotation = ci::Quatf());

	/*! Creates a rigid torus */
	//btRigidBody * create(btDynamicsWorld * world, float innerRadius = 5.0f, float outerRadius = 10.0f, int32_t segments = 16, const ci::Vec3f & position = ci::Vec3f::zero(), const ci::Quatf & rotation = ci::Quatf());

	/****** SOFT BODIES ******/

	/*! Create soft body from arbitrary collision shape */
	//btSoftBody * create(btDynamicsWorld * world, btSoftBodyWorldInfo & info, btCollisionShape * shape, const Vec3f & position = Vec3f::zero(), const Quatf & rotation = Quatf());

	/*! Creates soft body from triangle mesh */
	//btSoftBody * create(btDynamicsWorld * world, btSoftBodyWorldInfo & info, btBvhTriangleMeshShape * shape, const Vec3f & position = Vec3f::zero(), const Quatf & rotation = Quatf());

	/*! Creates soft concave mesh */
	//btSoftBody * create(btDynamicsWorld * world, btSoftBodyWorldInfo & info, btConvexHullShape * shape, const Vec3f & position = Vec3f::zero(), const Quatf & rotation = Quatf());

	/*! Create a soft box from AxisAlignedBox3f */
	//btSoftBody * create(btDynamicsWorld * world, btSoftBodyWorldInfo & info, const AxisAlignedBox3f & box, const Vec3f & position = Vec3f::zero(), const Quatf & rotation = Quatf());

	/*! Creates a soft box from two Rectfs */
	//btRigidBody * create(btDynamicsWorld * world, btSoftBodyWorldInfo & info, const Rectf & top, const Rectf & bottom, float height = 10.0f, const Vec3f & position = Vec3f::zero(), const Quatf & rotation = Quatf());

	/*! Creates soft box */
	btSoftBody * create(btDynamicsWorld * world, btSoftBodyWorldInfo & info, const ci::Vec3f & dimensions = ci::Vec3f(10.0f, 10.0f, 10.0f), const ci::Vec3f & position = ci::Vec3f::zero(), const ci::Quatf & rotation = ci::Quatf());

	/*! Creates a soft cylinder */
	//btRigidBody * create(btDynamicsWorld * world, float topRadius = 10.0f, float bottomRadius = 10.0f, float height = 20.0f, int32_t segments = 16, const Vec3f & position = Vec3f::zero(), const Quatf & rotation = Quatf());

	/*! Creates a soft sphere from Sphere */
	//btSoftBody * create(btDynamicsWorld * world, btSoftBodyWorldInfo & info, const Sphere & sphere, int32_t segments = 16, const Vec3f & position = Vec3f::zero(), const Quatf & rotation = Quatf());

	/*! Creates a soft sphere */
	btSoftBody * create(btDynamicsWorld * world, btSoftBodyWorldInfo & info, float radius = 10.0f, int32_t segments = 16, const ci::Vec3f & position = ci::Vec3f::zero(), const ci::Quatf & rotation = ci::Quatf());

	/*! Creates soft tetra box */
	btSoftBody * create(btDynamicsWorld * world, btSoftBodyWorldInfo & info, const TetraCube & tetraCube, const ci::Vec3f & dimensions = ci::Vec3f(10.0f, 10.0f, 10.0f), const ci::Vec3f & position = ci::Vec3f::zero(), const ci::Quatf & rotation = ci::Quatf());

	/*! Creates a soft torus */
	//btSoftBody * create(btDynamicsWorld * world, btSoftBodyWorldInfo & info, float innerRadius = 5.0f, float outerRadius = 10.0f, int32_t segments = 16, const Vec3f & position = Vec3f::zero(), const Quatf & rotation = Quatf());

} 
