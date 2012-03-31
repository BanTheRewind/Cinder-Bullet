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

// Includes
#include "CollisionObject.h"
#include "DynamicsWorld.h"
#include "BulletUtils.h"

namespace bullet
{

	class SoftObject
	{

	private:

		/*! Create soft body from arbitrary collision shape */
		//btSoftBody* create( btDynamicsWorld* world, btSoftBodyWorldInfo &info, btCollisionShape* shape, const Vec3f &position = Vec3f::zero(), const Quatf &rotation = Quatf() );

		/*! Creates soft body from triangle mesh */
		//btSoftBody* create( btDynamicsWorld* world, btSoftBodyWorldInfo &info, btBvhTriangleMeshShape* shape, const Vec3f &position = Vec3f::zero(), const Quatf &rotation = Quatf() );

		/*! Creates soft concave mesh */
		//btSoftBody* create( btDynamicsWorld* world, btSoftBodyWorldInfo &info, btConvexHullShape* shape, const Vec3f &position = Vec3f::zero(), const Quatf &rotation = Quatf() );

		/*! Create a soft box from AxisAlignedBox3f */
		//btSoftBody* create( btDynamicsWorld* world, btSoftBodyWorldInfo &info, const AxisAlignedBox3f &box, const Vec3f &position = Vec3f::zero(), const Quatf &rotation = Quatf() );

		/*! Creates a soft box from two Rectfs */
		//btRigidBody* create( btDynamicsWorld* world, btSoftBodyWorldInfo &info, const Rectf &top, const Rectf &bottom, float height = 10.0f, const Vec3f &position = Vec3f::zero(), const Quatf &rotation = Quatf() );

		/*! Creates soft box */
		//static btSoftBody* create( btDynamicsWorld* world, btSoftBodyWorldInfo &info, const ci::Vec3f &dimensions = ci::Vec3f( 10.0f, 10.0f, 10.0f ), 
			//const ci::Vec3f &position = ci::Vec3f::zero(), const ci::Quatf &rotation = ci::Quatf() );

		/*! Creates a soft cylinder */
		//btRigidBody* create( btDynamicsWorld* world, float topRadius = 10.0f, float bottomRadius = 10.0f, float height = 20.0f, int32_t segments = 16, const Vec3f &position = Vec3f::zero(), const Quatf &rotation = Quatf() );

		/*! Creates a soft sphere from Sphere */
		//btSoftBody* create( btDynamicsWorld* world, btSoftBodyWorldInfo &info, const Sphere &sphere, int32_t segments = 16, const Vec3f &position = Vec3f::zero(), const Quatf &rotation = Quatf() );

		/*! Creates a soft sphere */
		//static btSoftBody* create( btDynamicsWorld* world, btSoftBodyWorldInfo &info, float radius = 10.0f, int32_t segments = 16, 
			//const ci::Vec3f &position = ci::Vec3f::zero(), const ci::Quatf &rotation = ci::Quatf() );

		/*! Creates soft tetra box */
		/*static btSoftBody* create( btDynamicsWorld* world, btSoftBodyWorldInfo &info, const TetraCube &tetraCube, 
			const ci::Vec3f &dimensions = ci::Vec3f( 10.0f, 10.0f, 10.0f ), const ci::Vec3f &position = ci::Vec3f::zero(), 
			const ci::Quatf &rotation = ci::Quatf() );*/

		/*! Creates a soft torus */
		//static btSoftBody* create( btDynamicsWorld* world, btSoftBodyWorldInfo &info, float innerRadius = 5.0f, float outerRadius = 10.0f, int32_t segments = 16, const Vec3f &position = Vec3f::zero(), const Quatf &rotation = Quatf() );

		friend class CollisionObjectBase;

	};

	/*class SoftBox : public SoftObject 
	{
	public:
		SoftBox( DynamicsWorldRef world, const ci::Vec3f &dimensions = ci::Vec3f( 10.0f, 10.0f, 10.0f ), 
			const ci::Vec3f &position = ci::Vec3f::zero(), const ci::Quatf &rotation = ci::Quatf() );
	};

	class SoftSphere : public SoftObject 
	{
	public:
		SoftSphere( DynamicsWorldRef world, float radius = 10.0f, int32_t segments = 16, 
			const ci::Vec3f &position = ci::Vec3f::zero(), const ci::Quatf &rotation = ci::Quatf() );
	};

	class SoftTetraBox : public SoftObject 
	{
	public:
		SoftTetraBox( DynamicsWorldRef world, const ci::Vec3f &dimensions = ci::Vec3f( 10.0f, 10.0f, 10.0f ), 
			const ci::Vec3f &position = ci::Vec3f::zero(), const ci::Quatf &rotation = ci::Quatf() );
	private:
		TetraCube mTetraCube;
	};*/

}
