/*
* CinderBullet originally created by Peter Holzkorn on 2/16/10
* 
* Copyright (c) 2013, Ban the Rewind
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

#include "boost/signals2.hpp"
#include "boost/ptr_container/ptr_container.hpp"
#include "bullet/BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "cinder/gl/gl.h"
#include "cinder/TriMesh.h"

#include "Constraint.h"
#include "CollisionObject.h"

namespace bullet {
	typedef boost::ptr_vector<CollisionObject>		CollisionObjectList;
	typedef CollisionObjectList::pointer			CollisionObjectRef;
	typedef CollisionObjectList::const_iterator		ConstIter;
	typedef CollisionObjectList::iterator			Iter;

	typedef std::shared_ptr<class DynamicsWorld>	DynamicsWorldRef;

	class DynamicsWorld
	{
	public:
		~DynamicsWorld();

		Iter										begin();
		ConstIter									begin() const;
		void										clear();
		Iter										end();
		ConstIter									end() const;
		Iter										erase( Iter pos );
		Iter										erase( CollisionObjectRef &object );
		Iter										find( CollisionObjectRef &object );
		ConstIter									find( const CollisionObjectRef &object ) const;
		CollisionObjectRef							pushBack( CollisionObject *object );

		btBroadphaseInterface*						getBroadphase();
		btDefaultCollisionConfiguration*			getCollisionConfiguration();
		btCollisionDispatcher*						getDispatcher();
		btSoftBodyWorldInfo&						getInfo();
		const btSoftBodyWorldInfo&					getInfo() const;
		CollisionObjectList&						getObjects();
		const CollisionObjectList&					getObjects() const;
		uint32_t									getNumObjects();
		const uint32_t								getNumObjects() const;
		btConstraintSolver*							getSolver();
		btDiscreteDynamicsWorld*					getWorld();

		void										addConstraint( const Constraint &constraint, float clamping = 30.0f, float tau = 0.001f );
		bool										intersects( const ci::Ray &ray, float farClip, Constraint *constraint );
		void										removeConstraint( const Constraint &constraint );

		void										setInfo( const btSoftBodyWorldInfo &info );

		void										update( float step = 0.016667f );

		template<typename T, typename Y>
		inline uint32_t	addCollisionCallback( T callback, Y* callbackObject )
		{
			uint32_t id = mCallbacks.empty() ? 0 : mCallbacks.rbegin()->first + 1;
			mCallbacks.insert( std::make_pair( id, CallbackRef( new Callback( mSignal.connect( std::bind( callback, callbackObject, std::placeholders::_1, std::placeholders::_2 ) ) ) ) ) );
			return id;
		}

		void										removeCallback( uint32_t id );
	private:
		typedef boost::signals2::connection			Callback;
		typedef std::shared_ptr<Callback>			CallbackRef;
		typedef std::map<uint32_t, CallbackRef>		CallbackList;

		friend DynamicsWorldRef						createWorld();

		DynamicsWorld();

		btRigidBody*								toBulletRigidBody( Iter pos );
		btRigidBody*								toBulletRigidBody( CollisionObject *object );
		btSoftBody*									toBulletSoftBody( Iter pos );
		btSoftBody*									toBulletSoftBody( CollisionObject *object );

		double										mElapsedSeconds;

		uint32_t									mNumObjects;
		CollisionObjectList							mObjects;

		btBroadphaseInterface						*mBroadphase;
		btCollisionDispatcher						*mDispatcher;
		btSoftBodyWorldInfo							mSoftBodyWorldInfo;
		btConstraintSolver							*mSolver;
		btDefaultCollisionConfiguration				*mCollisionConfiguration;
		btSoftRigidDynamicsWorld					*mWorld;

		std::map<btCollisionObject*, btManifoldPoint>	mCollisions;

		CallbackList									mCallbacks;
		boost::signals2::signal<void ( btCollisionObject*, const btManifoldPoint& )>	mSignal;
	};

	DynamicsWorldRef								createWorld();

	CollisionObjectRef								createRigidBox( const DynamicsWorldRef &world, const ci::Vec3f &dimensions = ci::Vec3f::one(), 
																	float mass = 1.0f, const ci::Vec3f &position = ci::Vec3f::zero(), 
																	const ci::Quatf &rotation = ci::Quatf() );
	CollisionObjectRef								createRigidCone( const DynamicsWorldRef &world, float radius = 1.0f, float height = 1.0f, 
																		float mass = 1.0f, const ci::Vec3f &position = ci::Vec3f::zero(), const ci::Quatf &rotation = ci::Quatf() );
	CollisionObjectRef								createRigidCylinder( const DynamicsWorldRef &world, const ci::Vec3f &scale = ci::Vec3f::one(), 
																		float mass = 1.0f, const ci::Vec3f &position = ci::Vec3f::zero(), 
																		const ci::Quatf &rotation = ci::Quatf() );
	CollisionObjectRef								createRigidHull( const DynamicsWorldRef &world, const ci::TriMesh &mesh, const ci::Vec3f &scale = ci::Vec3f::one(), 
																	float mass = 1.0f, const ci::Vec3f &position = ci::Vec3f::zero(), 
																	const ci::Quatf &rotation = ci::Quatf() );
	CollisionObjectRef								createRigidMesh( const DynamicsWorldRef &world, const ci::TriMesh &mesh, const ci::Vec3f &scale = ci::Vec3f::one(), 
																	float margin = 0.05f, float mass = 1.0f, 
																	const ci::Vec3f &position = ci::Vec3f::zero(), 
																	const ci::Quatf &rotation = ci::Quatf() );
	CollisionObjectRef								createRigidSphere( const DynamicsWorldRef &world, float radius = 10.0f, 
																		float mass = 1.0f, const ci::Vec3f &position = ci::Vec3f::zero(), 
																		const ci::Quatf &rotation = ci::Quatf() );
	CollisionObjectRef								createRigidTerrain( const DynamicsWorldRef &world, const ci::Channel32f &heightField, float minHeight = -1.0f, 
																		float maxHeight = 1.0f, const ci::Vec3f &scale = ci::Vec3f::one(), float mass = 1.0f, 
																		const ci::Vec3f &position = ci::Vec3f::zero(), const ci::Quatf &rotation = ci::Quatf() );

	CollisionObjectRef								createSoftCloth( const DynamicsWorldRef &world, const ci::Vec2f &size = ci::Vec2f::one() * 10.0f, 
																	const ci::Vec2i &resolution = ci::Vec2i( 8, 8 ), int32_t corners = 0, 
																	const ci::Vec3f &position = ci::Vec3f::zero(), const ci::Quatf &rotation = ci::Quatf() );
	CollisionObjectRef								createSoftHull( const DynamicsWorldRef &world, const ci::TriMesh &mesh, const ci::Vec3f &scale = ci::Vec3f::one(), 
																	const ci::Vec3f &position = ci::Vec3f::zero(), const ci::Quatf &rotation = ci::Quatf() );
	CollisionObjectRef								createSoftMesh( const DynamicsWorldRef &world, const ci::TriMesh &mesh, const ci::Vec3f &scale = ci::Vec3f::one(), 
																	const ci::Vec3f &position = ci::Vec3f::zero(), const ci::Quatf &rotation = ci::Quatf() );

	btRigidBody*									toBulletRigidBody( Iter pos );
	btRigidBody*									toBulletRigidBody( const CollisionObjectRef &object );
	btSoftBody*										toBulletSoftBody( Iter pos );
	btSoftBody*										toBulletSoftBody( const CollisionObjectRef &object );

	ci::TriMesh										calcTriMesh( Iter pos );
	ci::TriMesh										calcTriMesh( const CollisionObjectRef &object );
}
