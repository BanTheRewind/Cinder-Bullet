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
#include "Utilities.h"

namespace bullet
{

	class RigidBody
	{
	protected:
		static btRigidBody*	create( btCollisionShape* shape, float mass, const ci::Vec3f &position, const ci::Quatf &rotation );
		static btRigidBody*	createBox( const ci::Vec3f &size, float mass, const ci::Vec3f &position, const ci::Quatf &rotation );
		static btRigidBody*	createCone( float radius, float height, int32_t segments, float mass, const ci::Vec3f &position, const ci::Quatf &rotation );
		static btRigidBody*	createCylinder( const ci::Vec3f &scale, int32_t segments, float mass, const ci::Vec3f &position, const ci::Quatf &rotation );
		static btRigidBody*	createHull( btConvexHullShape* shape, float mass, const ci::Vec3f &position, const ci::Quatf &rotation );
		static btRigidBody*	createMesh( btBvhTriangleMeshShape* shape, float mass, const ci::Vec3f &position, const ci::Quatf &rotation );
		static btRigidBody*	createSphere( float radius, float mass, const ci::Vec3f &position, const ci::Quatf &rotation );
		static btRigidBody*	createTerrain( btHeightfieldTerrainShape* shape, float mass, const ci::Vec3f &position, const ci::Quatf &rotation );
	};

	class RigidBox : public CollisionObject, public RigidBody
	{
	protected:
		RigidBox( const ci::Vec3f &dimensions, float mass, const ci::Vec3f &position, const ci::Quatf &rotation );
		friend CollisionObjectRef	createRigidBox( const DynamicsWorldRef &world, const ci::Vec3f &dimensions, float mass, 
													const ci::Vec3f &position, const ci::Quatf &rotation );
	};

	class RigidCone : public CollisionObject, public RigidBody 
	{
	protected:
		RigidCone( float radius, float height, int32_t segments, float mass, const ci::Vec3f &position, const ci::Quatf &rotation );
		friend CollisionObjectRef	createRigidCone( const DynamicsWorldRef &world, float radius, float height, int32_t segments, 
													float mass, const ci::Vec3f &position, const ci::Quatf &rotation );
	};

	class RigidCylinder : public CollisionObject, public RigidBody 
	{
	protected:
		RigidCylinder( const ci::Vec3f &scale, int32_t segments, float mass, const ci::Vec3f &position, const ci::Quatf &rotation );
		friend CollisionObjectRef	createRigidCylinder( const DynamicsWorldRef &world, const ci::Vec3f &scale, int32_t segments, 
														float mass, const ci::Vec3f &position, const ci::Quatf &rotation );
	};

	class RigidHull : public CollisionObject, public RigidBody 
	{
	protected:
		RigidHull( const ci::TriMesh &mesh, const ci::Vec3f &scale, float mass, const ci::Vec3f &position, const ci::Quatf &rotation );
		friend CollisionObjectRef	createRigidHull( const DynamicsWorldRef &world, const ci::TriMesh &mesh, const ci::Vec3f &scale, 
													float mass, const ci::Vec3f &position, const ci::Quatf &rotation );
	};

	class RigidMesh : public CollisionObject, public RigidBody 
	{
	protected:
		RigidMesh( const ci::TriMesh &mesh, const ci::Vec3f &scale, float margin, float mass, const ci::Vec3f &position, const ci::Quatf &rotation );
		friend CollisionObjectRef	createRigidMesh( const DynamicsWorldRef &world, const ci::TriMesh &mesh, const ci::Vec3f &scale, 
													float margin, float mass, const ci::Vec3f &position, const ci::Quatf &rotation );
	};

	class RigidSphere : public CollisionObject, public RigidBody 
	{
	protected:
		RigidSphere( float radius, int32_t segments, float mass, const ci::Vec3f &position, const ci::Quatf &rotation );
		friend CollisionObjectRef	createRigidSphere( const DynamicsWorldRef &world, float radius, int32_t segments, float mass, 
														const ci::Vec3f &position, const ci::Quatf &rotation );
	};

	class RigidTerrain : public CollisionObject, public RigidBody 
	{
	protected:
		RigidTerrain( const ci::Channel32f &heightField, float minHeight, float maxHeight, const ci::Vec3f &scale, float mass, 
			const ci::Vec3f &position, const ci::Quatf &rotation );
		friend CollisionObjectRef	createRigidTerrain( const DynamicsWorldRef &world, const ci::Channel32f &heightField, float minHeight, float maxHeight, 
														const ci::Vec3f &scale, float mass, const ci::Vec3f &position, const ci::Quatf &rotation );
		ci::Channel32f				mChannel;
		void						readChannelData();
	};

}
