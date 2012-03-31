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

#if defined( CINDER_MSW )
	#include "ppl.h"
#endif
#include "boost/ptr_container/ptr_vector.hpp"

namespace bullet {

	typedef boost::ptr_vector<CollisionObjectBase> CollisionObjectList;
	typedef boost::ptr_vector<CollisionObjectBase>::reference CollisionObject;
	typedef boost::ptr_vector<CollisionObjectBase>::iterator Iter;

	typedef std::shared_ptr<class DynamicsWorld> DynamicsWorldRef;

	// Bullet physics world manager
	class DynamicsWorld
	{

	public:

		~DynamicsWorld();

	private:

		static DynamicsWorldRef								getInstance();
		DynamicsWorld();

		friend Iter											begin();
		Iter												begin();
		friend Iter											end();
		Iter												end();
		friend Iter											erase( Iter pos );
		Iter												erase( Iter pos );

		friend CollisionObject								createRigidBox( const ci::Vec3f &dimensions, const ci::Vec3f &position, const ci::Quatf &rotation );
		CollisionObject										createRigidBox( const ci::Vec3f &dimensions, const ci::Vec3f &position, const ci::Quatf &rotation );
		friend CollisionObject								createRigidCylinder( float topRadius, float bottomRadius, float height, int32_t segments, 
																				const ci::Vec3f &position, const ci::Quatf &rotation );
		CollisionObject										createRigidCylinder( float topRadius, float bottomRadius, float height, int32_t segments, 
																				const ci::Vec3f &position, const ci::Quatf &rotation );
		friend CollisionObject								createRigidHull( const ci::TriMesh &mesh, const ci::Vec3f &scale, const ci::Vec3f &position, 
																			const ci::Quatf &rotation );
		CollisionObject										createRigidHull( const ci::TriMesh &mesh, const ci::Vec3f &scale, const ci::Vec3f &position, 
																			const ci::Quatf &rotation );
		friend CollisionObject								createRigidMesh( const ci::TriMesh &mesh, const ci::Vec3f &scale, float margin, 
																			const ci::Vec3f &position, const ci::Quatf &rotation );
		CollisionObject										createRigidMesh( const ci::TriMesh &mesh, const ci::Vec3f &scale, float margin, 
																			const ci::Vec3f &position, const ci::Quatf &rotation );
		friend CollisionObject								createRigidSphere( float radius, int32_t segments, const ci::Vec3f &position, const ci::Quatf &rotation );
		CollisionObject										createRigidSphere( float radius, int32_t segments, const ci::Vec3f &position, const ci::Quatf &rotation );
		friend CollisionObject								createRigidTerrain( const ci::Surface32f &heightField, int32_t stickWidth, int32_t stickLength, 
																				float minHeight, float maxHeight, int32_t upAxis, const ci::Vec3f &scale, 
																				const ci::Vec3f &position, const ci::Quatf &rotation );
		CollisionObject										createRigidTerrain( const ci::Surface32f &heightField, int32_t stickWidth, int32_t stickLength, 
																				float minHeight, float maxHeight, int32_t upAxis, const ci::Vec3f &scale, 
																				const ci::Vec3f &position, const ci::Quatf &rotation );

		friend btBroadphaseInterface*						getBroadphase();
		btBroadphaseInterface*								getBroadphase();
			
		friend btSoftBodyRigidBodyCollisionConfiguration*	getCollisionConfiguration();
		btSoftBodyRigidBodyCollisionConfiguration*			getCollisionConfiguration();
			
		friend btCollisionDispatcher*						getDispatcher();
		btCollisionDispatcher*								getDispatcher();
			
		friend btSoftBodyWorldInfo&							getInfo();
		btSoftBodyWorldInfo&								getInfo();

		friend CollisionObjectList&							getObjects();
		CollisionObjectList&								getObjects();
			
		friend uint32_t										getNumObjects();
		uint32_t											getNumObjects();

		friend btConstraintSolver*							getSolver();
		btConstraintSolver*									getSolver();

		friend btDynamicsWorld*								getWorld();
		btDynamicsWorld*									getWorld();
			
		friend void											setInfo( const btSoftBodyWorldInfo &info );
		void												setInfo( const btSoftBodyWorldInfo &info );

		friend void											update();
		void												update();

		double												mElapsedSeconds;

		uint32_t											mNumObjects;
		CollisionObjectList									mObjects;

		btBroadphaseInterface								*mBroadphase;
		btCollisionDispatcher								*mDispatcher;
		btSoftBodyWorldInfo									mSoftBodyWorldInfo;
		btConstraintSolver									*mSolver;
		btSoftBodyRigidBodyCollisionConfiguration			*mCollisionConfiguration;
		btDynamicsWorld										*mWorld;

	};

	Iter													begin();
	Iter													end();
	Iter													erase( Iter pos );

	CollisionObject											createRigidBox( const ci::Vec3f &dimensions = ci::Vec3f::one() * 10.0f, 
																			const ci::Vec3f &position = ci::Vec3f::zero(), 
																			const ci::Quatf &rotation = ci::Quatf() );
	CollisionObject											createRigidCylinder( float topRadius = 10.0f, float bottomRadius = 10.0f, 
																				float height = 20.0f, int32_t segments = 16, 
																				const ci::Vec3f &position = ci::Vec3f::zero(), 
																				const ci::Quatf &rotation = ci::Quatf() );
	CollisionObject											createRigidHull( const ci::TriMesh &mesh, const ci::Vec3f &scale = ci::Vec3f::one(), 
																			const ci::Vec3f &position = ci::Vec3f::zero(), 
																			const ci::Quatf &rotation = ci::Quatf() );
	CollisionObject											createRigidMesh( const ci::TriMesh &mesh, const ci::Vec3f &scale = ci::Vec3f::one(), 
																			float margin = 0.05f, const ci::Vec3f &position = ci::Vec3f::zero(), 
																			const ci::Quatf &rotation = ci::Quatf() );
	CollisionObject											createRigidSphere( float radius = 10.0f, int32_t segments = 16, 
																				const ci::Vec3f &position = ci::Vec3f::zero(), 
																				const ci::Quatf &rotation = ci::Quatf() );
	CollisionObject											createRigidTerrain( const ci::Surface32f &heightField, int32_t stickWidth, 
																				int32_t stickLength, float minHeight = -200.0f, float maxHeight = 200.0f, 
																				int32_t upAxis = 1, const ci::Vec3f &scale = ci::Vec3f( 1.0f, 100.0f, 1.0f ), 
																				const ci::Vec3f &position = ci::Vec3f::zero(), 
																				const ci::Quatf &rotation = ci::Quatf() );

	btBroadphaseInterface*									getBroadphase();
	btSoftBodyRigidBodyCollisionConfiguration*				getCollisionConfiguration();
	btCollisionDispatcher*									getDispatcher();
	btSoftBodyWorldInfo&									getInfo();
	CollisionObjectList&									getObjects();
	uint32_t												getNumObjects();
	btConstraintSolver*										getSolver();
	btDynamicsWorld*										getWorld();

	void													setInfo( const btSoftBodyWorldInfo &info );

	void													update();

}
