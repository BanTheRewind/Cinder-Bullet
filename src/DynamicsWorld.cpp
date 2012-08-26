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

#include "DynamicsWorld.h"

#include "cinder/app/App.h"
#include "cinder/Utilities.h"

#include "RigidBody.h"
#include "SoftBody.h"

#include "bullet/BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"

namespace bullet {
	using namespace ci;
	using namespace ci::app;
	using namespace std;

	/////////////////////////////////////////////////////////////////////////////////////////////////

	DynamicsWorld::DynamicsWorld()
	{
		mCollisionConfiguration	= new btSoftBodyRigidBodyCollisionConfiguration();
		mDispatcher				= new btCollisionDispatcher( mCollisionConfiguration );
		// TODO make these settings configurable / updateable
		mBroadphase				= new btAxisSweep3( btVector3( -1000.0f, -1000.0f, -1000.0f ), btVector3( 1000.0f, 1000.0f, 1000.0f ), 32766 );
		mSolver					= new btSequentialImpulseConstraintSolver();
	
		mSoftBodyWorldInfo.air_density		= 1.2f;
		mSoftBodyWorldInfo.m_broadphase		= mBroadphase;
		mSoftBodyWorldInfo.m_dispatcher		= mDispatcher;
		mSoftBodyWorldInfo.m_gravity.setValue( 0.0f, -10.0f, 0.0f );
		mSoftBodyWorldInfo.water_density	= 0.0f;
		mSoftBodyWorldInfo.water_offset		= 0.0f;
		mSoftBodyWorldInfo.water_normal		= btVector3( 0.0f, 0.0f, 0.0f );
		mSoftBodyWorldInfo.m_sparsesdf.Initialize();
		btSoftBodySolver *softBodySolver	= 0;
		mWorld = new btSoftRigidDynamicsWorld( mDispatcher, mBroadphase, mSolver, mCollisionConfiguration, softBodySolver );
		mWorld->setGravity( btVector3( 0.0f, -10.0f, 0.0f ) );
		mWorld->getDispatchInfo().m_enableSPU = true;

		mElapsedSeconds = getElapsedSeconds();
		mNumObjects		= 0;
	}

	DynamicsWorld::~DynamicsWorld()
	{
		mSoftBodyWorldInfo.m_sparsesdf.GarbageCollect();
		if ( mBroadphase != 0 ) {
			delete mBroadphase;
		}
		if ( mCollisionConfiguration != 0 ) {
			delete mCollisionConfiguration;
		}
		if ( mDispatcher != 0 ) {
			delete mDispatcher;
		}
		if ( mSolver != 0 ) {
			delete mSolver;
		}
		if ( mWorld != 0 ) {
			delete mWorld;
		}
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////

	Iter DynamicsWorld::begin()
	{
		return mObjects.begin();
	}

	ConstIter DynamicsWorld::begin() const
	{
		return mObjects.begin();
	}

	void DynamicsWorld::clear()
	{
		for ( bullet::Iter object = mObjects.begin(); object != mObjects.end(); ) {
			object = erase( object );
		}
	}
	
	Iter DynamicsWorld::end()
	{
		return mObjects.end();
	}
	
	ConstIter DynamicsWorld::end() const
	{
		return mObjects.end();
	}
	
	Iter DynamicsWorld::erase( Iter pos )
	{
		if ( pos->isRigidBody() ) {
			mWorld->removeRigidBody( toBulletRigidBody( pos ) );
		} else if ( pos->isSoftBody() ) {
			( (btSoftRigidDynamicsWorld*)mWorld )->removeSoftBody( toBulletSoftBody( pos ) );
		}
		return mObjects.erase( pos );
	}

	Iter DynamicsWorld::erase( CollisionObjectRef &object )
	{
		Iter pos = find( object );
		return erase( pos );
	}

	Iter DynamicsWorld::find( CollisionObjectRef &object )
	{
		Iter iter;
		for ( Iter objectIt = mObjects.begin(); objectIt != mObjects.end(); ++objectIt ) {
			if ( object == &*objectIt ) {
				return objectIt;
			}
		}
		// TO DO: throw exception here
		return iter;
	}

	ConstIter DynamicsWorld::find( const CollisionObjectRef &object ) const
	{
		ConstIter iter;
		for ( ConstIter objectIt = mObjects.begin(); objectIt != mObjects.end(); ++objectIt ) {
			if ( object == &*objectIt ) {
				return objectIt;
			}
		}
		// TO DO: throw exception here
		return iter;
	}

	CollisionObjectRef DynamicsWorld::pushBack( CollisionObject *object )
	{
		mObjects.push_back( object );
		if ( object->isRigidBody() ) {
			btRigidBody* body = toBulletRigidBody( object );
			mWorld->addRigidBody( toBulletRigidBody( object ) );
		} else if ( object->isSoftBody() ) {
			btSoftBody* body = toBulletSoftBody( object );
			mWorld->addSoftBody( body );
		}
		return &mObjects[ mObjects.size() - 1 ];
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////

	btBroadphaseInterface* DynamicsWorld::getBroadphase() 
	{ 
		return mBroadphase; 
	}

	btDefaultCollisionConfiguration* DynamicsWorld::getCollisionConfiguration() 
	{ 
		return mCollisionConfiguration; 
	}

	btCollisionDispatcher* DynamicsWorld::getDispatcher() 
	{ 
		return mDispatcher; 
	}

	btSoftBodyWorldInfo& DynamicsWorld::getInfo() 
	{ 
		return mSoftBodyWorldInfo; 
	}

	const btSoftBodyWorldInfo& DynamicsWorld::getInfo() const
	{ 
		return mSoftBodyWorldInfo; 
	}

	uint32_t DynamicsWorld::getNumObjects()
	{
		return mNumObjects;
	}

	const uint32_t DynamicsWorld::getNumObjects() const
	{
		return mNumObjects;
	}

	CollisionObjectList& DynamicsWorld::getObjects()
	{
		return mObjects;
	}

	const CollisionObjectList& DynamicsWorld::getObjects() const
	{
		return mObjects;
	}

	btConstraintSolver* DynamicsWorld::getSolver() 
	{ 
		return mSolver; 
	}

	btDiscreteDynamicsWorld* DynamicsWorld::getWorld() 
	{ 
		return mWorld; 
	}

	void DynamicsWorld::setInfo( const btSoftBodyWorldInfo &info )
	{
		mSoftBodyWorldInfo = info;
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////

	void DynamicsWorld::addConstraint( const Constraint &constraint, float clamping, float tau )
	{
		mWorld->addConstraint( constraint.mConstraint );
		constraint.mConstraint->m_setting.m_impulseClamp	= clamping;
		constraint.mConstraint->m_setting.m_tau				= tau;
	}

	bool DynamicsWorld::intersects( const Ray &ray, float farClip, Constraint *constraint )
	{
		btVector3 rayFrom	= toBulletVector3( ray.getOrigin() );
		btVector3 rayTo		= toBulletVector3( ray.calcPosition( farClip ) );

		btCollisionWorld::ClosestRayResultCallback rayCallback( rayFrom, rayTo );
		mWorld->rayTest( rayFrom, rayTo, rayCallback );

		if ( rayCallback.hasHit() ) {
			btRigidBody* collisionBody = btRigidBody::upcast( rayCallback.m_collisionObject );
			if ( collisionBody ) {
				btVector3 position	= rayCallback.m_hitPointWorld;
				btVector3 pivot		= collisionBody->getCenterOfMassTransform().inverse() * position;

				constraint->mConstraint = new btPoint2PointConstraint( *collisionBody, pivot );
				constraint->mDistance	= ( position - rayFrom ).length();
				constraint->mPosition	= fromBulletVector3( rayTo );
				
				return true;
			}
		}
		return false;
	}

	void DynamicsWorld::removeConstraint( const Constraint &constraint )
	{
		mWorld->removeConstraint( constraint.mConstraint );
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////

	void DynamicsWorld::update( float step )
	{
		uint32_t numObjects = mWorld->getCollisionObjectArray().size();
		if ( mNumObjects != numObjects ) {
			if ( numObjects <= 0 ) {
				return;
			}

			for ( uint32_t i = 0; i < numObjects; i++ ) {
				btCollisionObject* collisionObject = mWorld->getCollisionObjectArray()[ i ];
				btRigidBody* rigidBody = btRigidBody::upcast( collisionObject );
				if ( rigidBody ) {
					rigidBody->activate( true );
				} else {
					btSoftBody* softBody = btSoftBody::upcast( collisionObject );
					if ( softBody ) {
						softBody->activate( true );
					}
				}
			}

		}

		for ( Iter iter = mObjects.begin(); iter != mObjects.end(); ++iter ) {
			iter->update();
		}

		mNumObjects = numObjects;

		mWorld->stepSimulation( 1.0f, 10, step );
		mSoftBodyWorldInfo.m_sparsesdf.GarbageCollect();
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////

	
	btRigidBody* DynamicsWorld::toBulletRigidBody( Iter pos )
	{
		return (btRigidBody*)( pos->getBulletBody() );
	}

	btRigidBody* DynamicsWorld::toBulletRigidBody( CollisionObject *object )
	{
		return (btRigidBody*)( object->getBulletBody() );
	}

	btSoftBody* DynamicsWorld::toBulletSoftBody( Iter pos )
	{
		return (btSoftBody*)( pos->getBulletBody() );
	}

	btSoftBody* DynamicsWorld::toBulletSoftBody( CollisionObject *object )
	{
		return (btSoftBody*)( object->getBulletBody() );
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////

	DynamicsWorldRef createWorld()
	{
		return DynamicsWorldRef( new DynamicsWorld() );
	}

	CollisionObjectRef createRigidBox( const DynamicsWorldRef &world, const Vec3f &dimensions, float mass, const Vec3f &position, const Quatf &rotation )
	{
		return world->pushBack( new RigidBox( dimensions, mass, position, rotation ) );
	}

	CollisionObjectRef createRigidCone( const DynamicsWorldRef &world, float radius, float height, float mass, 
		const Vec3f &position, const Quatf &rotation )
	{
		return world->pushBack( new RigidCone( radius, height, mass, position, rotation ) );
	}

	CollisionObjectRef createRigidCylinder( const DynamicsWorldRef &world, const Vec3f &scale, float mass, 
		const Vec3f &position, const Quatf &rotation )
	{
		return world->pushBack( new RigidCylinder( scale, mass, position, rotation ) );
	}

	CollisionObjectRef createRigidHull( const DynamicsWorldRef &world, const TriMesh &mesh, const Vec3f &scale, float mass, const Vec3f &position, const Quatf &rotation )
	{
		return world->pushBack( new RigidHull( mesh, scale, mass, position, rotation ) );
	}
	CollisionObjectRef createRigidMesh( const DynamicsWorldRef &world, const TriMesh &mesh, const Vec3f &scale, float margin, float mass, 
		const Vec3f &position, const Quatf &rotation )
	{
		return world->pushBack( new RigidMesh( mesh, scale, margin, mass, position, rotation ) );
	}

	CollisionObjectRef createRigidSphere( const DynamicsWorldRef &world, float radius, float mass, const Vec3f &position, const Quatf &rotation )
	{
		return world->pushBack( new RigidSphere( radius, mass, position, rotation ) );
	}

	CollisionObjectRef createRigidTerrain( const DynamicsWorldRef &world, const Channel32f &heightField, float minHeight, float maxHeight, 
		const Vec3f &scale, float mass, const Vec3f &position, const Quatf &rotation )
	{
		return world->pushBack( new RigidTerrain( heightField, minHeight, maxHeight, scale, mass, position, rotation ) );
	}

	CollisionObjectRef createSoftCloth( const DynamicsWorldRef &world, const Vec2f &size, const Vec2i &resolution, int32_t corners, 
		const Vec3f &position, const Quatf &rotation )
	{
		return world->pushBack( new SoftCloth( world->getInfo(), size, resolution, corners, position, rotation ) );
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////

	btRigidBody* toBulletRigidBody( Iter pos )
	{
		return (btRigidBody*)( pos->getBulletBody() );
	}
	
	btRigidBody* toBulletRigidBody( const CollisionObjectRef &object )
	{
		return (btRigidBody*)( object->getBulletBody() );
	}

	btSoftBody* toBulletSoftBody( Iter pos )
	{
		return (btSoftBody*)( pos->getBulletBody() );
	}

	btSoftBody* toBulletSoftBody( const CollisionObjectRef &object )
	{
		return (btSoftBody*)( object->getBulletBody() );
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////

	ci::TriMesh calcTriMesh( Iter pos )
	{
		TriMesh mesh;
		mesh.appendIndices( &pos->getIndices()[ 0 ], pos->getIndices().size() );
		const vector<Vec3f>& normals = pos->getNormals();
		for ( vector<Vec3f>::const_iterator iter = normals.begin(); iter != normals.end(); ++iter ) {
			mesh.appendNormal( *iter );
		}
		mesh.appendVertices( &pos->getPositions()[ 0 ], pos->getPositions().size() );
		const vector<Vec2f>& texCoords = pos->getTexCoords();
		for ( vector<Vec2f>::const_iterator iter = texCoords.begin(); iter != texCoords.end(); ++iter ) {
			mesh.appendTexCoord( *iter );
		}
		return mesh;
	}

	ci::TriMesh calcTriMesh( const CollisionObjectRef &object )
	{
		TriMesh mesh;
		mesh.appendIndices( &object->getIndices()[ 0 ], object->getIndices().size() );
		const vector<Vec3f>& normals = object->getNormals();
		for ( vector<Vec3f>::const_iterator iter = normals.begin(); iter != normals.end(); ++iter ) {
			mesh.appendNormal( *iter );
		}
		mesh.appendVertices( &object->getPositions()[ 0 ], object->getPositions().size() );
		const vector<Vec2f>& texCoords = object->getTexCoords();
		for ( vector<Vec2f>::const_iterator iter = texCoords.begin(); iter != texCoords.end(); ++iter ) {
			mesh.appendTexCoord( *iter );
		}
		return mesh;
	}
}
