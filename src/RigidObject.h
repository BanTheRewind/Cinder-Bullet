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

	class RigidObject : public CollisionObjectBase
	{

	protected:

		RigidObject( btDynamicsWorld* world, const ci::Vec3f &position = ci::Vec3f::zero(), const ci::Quatf &rotation = ci::Quatf() );

		/*! Creates rigid body from collision shape */
		btRigidBody* create( btDynamicsWorld* world, btCollisionShape* shape, const ci::Vec3f &position = ci::Vec3f::zero(), 
			const ci::Quatf &rotation = ci::Quatf() );

		/*! Create rigid body from triangle mesh */
		btRigidBody* create( btDynamicsWorld* world, btBvhTriangleMeshShape* shape, const ci::Vec3f &position = ci::Vec3f::zero(), 
			const ci::Quatf &rotation = ci::Quatf() );

		/*! Creates rigid body from convex hull shape */
		btRigidBody* create( btDynamicsWorld* world, btConvexHullShape* shape, const ci::Vec3f &position = ci::Vec3f::zero(), 
			const ci::Quatf &rotation = ci::Quatf() );

		/*! Creates a rigid box */
		btRigidBody* create( btDynamicsWorld* world, const ci::Vec3f &size = ci::Vec3f( 10.0f, 10.0f, 10.0f ), 
			const ci::Vec3f &position = ci::Vec3f::zero(), const ci::Quatf &rotation = ci::Quatf() );

		/*! Creates a rigid cylinder */
		btRigidBody* create( btDynamicsWorld* world, float topRadius = 10.0f, float bottomRadius = 10.0f, float height = 20.0f, 
			int32_t segments = 16, const ci::Vec3f &position = ci::Vec3f::zero(), const ci::Quatf &rotation = ci::Quatf() );

		/*! Creates a rigid sphere */
		btRigidBody* create( btDynamicsWorld* world, float radius = 10.0f, const ci::Vec3f &position = ci::Vec3f::zero(), 
			const ci::Quatf &rotation = ci::Quatf() );

		/*! Creates a rigid torus */
		//btRigidBody* create( btDynamicsWorld* world, float innerRadius = 5.0f, float outerRadius = 10.0f, int32_t segments = 16, const ci::Vec3f &position = ci::Vec3f::zero(), const ci::Quatf &rotation = ci::Quatf() );

	};

	class RigidBox : public RigidObject 
	{
	private:
		RigidBox( btDynamicsWorld* world, const ci::Vec3f &dimensions, const ci::Vec3f &position, const ci::Quatf &rotation );
		friend class	DynamicsWorld;
	};

	class RigidCylinder : public RigidObject 
	{

	private:
		RigidCylinder( btDynamicsWorld* world, float topRadius, float bottomRadius, float height, int32_t segments, 
			const ci::Vec3f &position, const ci::Quatf &rotation );
		friend class	DynamicsWorld;
	};

	class RigidHull : public RigidObject 
	{
	private:
		RigidHull( btDynamicsWorld* world, const ci::TriMesh &mesh, const ci::Vec3f &scale, 
			const ci::Vec3f &position, const ci::Quatf &rotation );
		friend class	DynamicsWorld;
	};

	class RigidMesh : public RigidObject 
	{
	private:
		RigidMesh( btDynamicsWorld* world, const ci::TriMesh &mesh, const ci::Vec3f &scale, float margin, 
			const ci::Vec3f &position, const ci::Quatf &rotation );
		friend class	DynamicsWorld;
	};

	class RigidSphere : public RigidObject 
	{
	private:
		RigidSphere( btDynamicsWorld* world, float radius, int32_t segments, const ci::Vec3f &position, const ci::Quatf &rotation );
		friend class	DynamicsWorld;
	};

	class RigidTerrain : public RigidObject 
	{
	private:
		RigidTerrain( btDynamicsWorld* world, const ci::Surface32f &heightField, int32_t stickWidth, int32_t stickLength, float minHeight, 
			float maxHeight, int32_t upAxis, const ci::Vec3f &scale, const ci::Vec3f &position, const ci::Quatf &rotation );
		friend class	DynamicsWorld;
	};

	/*! Rigid torus */
	/*class RigidTorus : public RigidObject 
	{
	public:
		static CollisionObjectBase create(btDynamicsWorld* world, float innerRadius = 5.0f, float outerRadius = 10.0f, int32_t segments = 16, const ci::Vec3f & position = ci::Vec3f::zero(), const ci::Quatf & rotation = ci::Quatf());
	private:
		RigidTorus(btDynamicsWorld* world, float innerRadius = 5.0f, float outerRadius = 10.0f, int32_t segments = 16, const ci::Vec3f & position = ci::Vec3f::zero(), const ci::Quatf & rotation = ci::Quatf());
	};*/

}
