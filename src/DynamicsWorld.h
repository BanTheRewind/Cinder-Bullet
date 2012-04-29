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

#include "cinder/gl/gl.h"

#if defined( CINDER_MSW )
	#include "ppl.h"
#endif
#include "boost/ptr_container/ptr_container.hpp"

namespace bullet {

	typedef boost::ptr_vector<CollisionObject>				CollisionObjectList;
	typedef CollisionObjectList::pointer					CollisionObjectRef;
	typedef CollisionObjectList::const_iterator				ConstIter;
	typedef CollisionObjectList::iterator					Iter;

	typedef std::shared_ptr<class DynamicsWorld>			DynamicsWorldRef;

	// Bullet physics world manager
	class DynamicsWorld
	{

	public:

		~DynamicsWorld();

		Iter												begin();
		ConstIter											begin() const;
		void												clear();
		Iter												end();
		ConstIter											end() const;
		Iter												erase( Iter pos );
		Iter												erase( CollisionObjectRef &object );
		Iter												find( CollisionObjectRef &object );
		ConstIter											find( const CollisionObjectRef &object ) const;
		CollisionObjectRef									pushBack( CollisionObject *object );

		btBroadphaseInterface*								getBroadphase();
		btDefaultCollisionConfiguration*					getCollisionConfiguration();
		btCollisionDispatcher*								getDispatcher();
		btSoftBodyWorldInfo&								getInfo();
		const btSoftBodyWorldInfo&							getInfo() const;
		CollisionObjectList&								getObjects();
		const CollisionObjectList&							getObjects() const;
		uint32_t											getNumObjects();
		const uint32_t										getNumObjects() const;
		btConstraintSolver*									getSolver();
		btDiscreteDynamicsWorld*							getWorld();

		void												setInfo( const btSoftBodyWorldInfo &info );

		void												update( float frameRate = 60.0f );

	private:

		friend DynamicsWorldRef								createWorld();
		static DynamicsWorldRef								create();

		DynamicsWorld();
		DynamicsWorld( DynamicsWorld const& );
		const DynamicsWorld&								operator=( DynamicsWorld const& );

		btRigidBody*										toBulletRigidBody( Iter pos );
		btRigidBody*										toBulletRigidBody( CollisionObject *object );
		btSoftBody*											toBulletSoftBody( Iter pos );
		btSoftBody*											toBulletSoftBody( CollisionObject *object );

		double												mElapsedSeconds;

		uint32_t											mNumObjects;
		CollisionObjectList									mObjects;

		btBroadphaseInterface								*mBroadphase;
		btCollisionDispatcher								*mDispatcher;
		btSoftBodyWorldInfo									mSoftBodyWorldInfo;
		btConstraintSolver									*mSolver;
		btDefaultCollisionConfiguration						*mCollisionConfiguration;
		btDiscreteDynamicsWorld								*mWorld;

	};

	DynamicsWorldRef										createWorld();

	CollisionObjectRef										createRigidBox( const DynamicsWorldRef &world, const ci::Vec3f &dimensions = ci::Vec3f::one(), 
																			float mass = 1.0f, const ci::Vec3f &position = ci::Vec3f::zero(), 
																			const ci::Quatf &rotation = ci::Quatf() );
	CollisionObjectRef										createRigidCone( const DynamicsWorldRef &world, float radius = 1.0f, float height = 1.0f, int32_t segments = 16, 
																			float mass = 1.0f, const ci::Vec3f &position = ci::Vec3f::zero(), const ci::Quatf &rotation = ci::Quatf() );
	CollisionObjectRef										createRigidCylinder( const DynamicsWorldRef &world, const ci::Vec3f &scale = ci::Vec3f::one(), int32_t segments = 16, 
																				float mass = 1.0f, const ci::Vec3f &position = ci::Vec3f::zero(), 
																				const ci::Quatf &rotation = ci::Quatf() );
	CollisionObjectRef										createRigidHull( const DynamicsWorldRef &world, const ci::TriMesh &mesh, const ci::Vec3f &scale = ci::Vec3f::one(), 
																			float mass = 1.0f, const ci::Vec3f &position = ci::Vec3f::zero(), 
																			const ci::Quatf &rotation = ci::Quatf() );
	CollisionObjectRef										createRigidMesh( const DynamicsWorldRef &world, const ci::TriMesh &mesh, const ci::Vec3f &scale = ci::Vec3f::one(), 
																			float margin = 0.05f, float mass = 1.0f, 
																			const ci::Vec3f &position = ci::Vec3f::zero(), 
																			const ci::Quatf &rotation = ci::Quatf() );
	CollisionObjectRef										createRigidSphere( const DynamicsWorldRef &world, float radius = 10.0f, int32_t segments = 16, 
																				float mass = 1.0f, const ci::Vec3f &position = ci::Vec3f::zero(), 
																				const ci::Quatf &rotation = ci::Quatf() );
	CollisionObjectRef										createStaticPlane( const DynamicsWorldRef &world, const ci::Vec3f &normal = ci::Vec3f( 0.0f, 1.0f, 0.0f ), 
																				float planeConstant = 0.0f, const ci::Vec3f &position = ci::Vec3f::zero(), 
																				const ci::Quatf &rotation = ci::Quatf() );
	CollisionObjectRef										createRigidTerrain( const DynamicsWorldRef &world, const ci::Channel32f &heightField, float minHeight = -1.0f, 
																				float maxHeight = 1.0f, const ci::Vec3f &scale = ci::Vec3f::one(), float mass = 1.0f, 
																				const ci::Vec3f &position = ci::Vec3f::zero(), const ci::Quatf &rotation = ci::Quatf() );

	btRigidBody*											toBulletRigidBody( const CollisionObjectRef &object );
	btSoftBody*												toBulletSoftBody( const CollisionObjectRef &object );

}
