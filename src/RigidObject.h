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

	class RigidBox : public CollisionObjectBase 
	{
	private:
		RigidBox( const ci::Vec3f &dimensions, const ci::Vec3f &position, const ci::Quatf &rotation );
		friend class	DynamicsWorld;
	};

	class RigidCylinder : public CollisionObjectBase 
	{

	private:
		RigidCylinder( float topRadius, float bottomRadius, float height, int32_t segments, const ci::Vec3f &position, const ci::Quatf &rotation );
		friend class	DynamicsWorld;
	};

	class RigidHull : public CollisionObjectBase 
	{
	private:
		RigidHull( const ci::TriMesh &mesh, const ci::Vec3f &scale, 
			const ci::Vec3f &position, const ci::Quatf &rotation );
		friend class	DynamicsWorld;
	};

	class RigidMesh : public CollisionObjectBase 
	{
	private:
		RigidMesh( const ci::TriMesh &mesh, const ci::Vec3f &scale, float margin, const ci::Vec3f &position, const ci::Quatf &rotation );
		friend class	DynamicsWorld;
	};

	class RigidSphere : public CollisionObjectBase 
	{
	private:
		RigidSphere( float radius, int32_t segments, const ci::Vec3f &position, const ci::Quatf &rotation );
		friend class	DynamicsWorld;
	};

	class RigidTerrain : public CollisionObjectBase 
	{
	private:
		RigidTerrain( const ci::Surface32f &heightField, int32_t stickWidth, int32_t stickLength, float minHeight, 
			float maxHeight, int32_t upAxis, const ci::Vec3f &scale, const ci::Vec3f &position, const ci::Quatf &rotation );
		friend class	DynamicsWorld;
	};

	class RigidTorus : public CollisionObjectBase 
	{
	private:
		RigidTorus( float innerRadius, float outerRadius, int32_t segments, const ci::Vec3f &position, const ci::Quatf &rotation );
		friend class	DynamicsWorld;
	};

	class RigidObject
	{
	private:
		static btRigidBody*	create( btCollisionShape* shape, float mass, const ci::Vec3f &position, const ci::Quatf &rotation );
		static btRigidBody*	createBox( const ci::Vec3f &size, const ci::Vec3f &position, const ci::Quatf &rotation );
		static btRigidBody*	createCylinder( float topRadius, float bottomRadius, float height, int32_t segments, 
			const ci::Vec3f &position, const ci::Quatf &rotation );
		static btRigidBody*	createHull( btConvexHullShape* shape, const ci::Vec3f &position, const ci::Quatf &rotation );
		static btRigidBody*	createMesh( btBvhTriangleMeshShape* shape, const ci::Vec3f &position, const ci::Quatf &rotation );
		static btRigidBody*	createSphere( float radius, const ci::Vec3f &position, const ci::Quatf &rotation );
		static btRigidBody*	createTerrain( btHeightfieldTerrainShape* shape, const ci::Vec3f &position, const ci::Quatf &rotation );
		
		friend class		RigidBox;
		friend class		RigidCylinder;
		friend class		RigidHull;
		friend class		RigidMesh;
		friend class		RigidSphere;
		friend class		RigidTerrain;
		friend class		RigidTorus;
	};

}
