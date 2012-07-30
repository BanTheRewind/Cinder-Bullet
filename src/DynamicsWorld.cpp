// Include header
#include "DynamicsWorld.h"

#include "cinder/app/App.h"

#include "bullet/BulletSoftBody/btSoftRigidDynamicsWorld.h"

#include "RigidBody.h"
#include "SoftBody.h"

namespace bullet {

	using namespace ci;
	using namespace ci::app;
	using namespace std;

	/////////////////////////////////////////////////////////////////////////////////////////////////

	DynamicsWorld::DynamicsWorld( const CameraPersp &camera )
	{

		mCamera = camera;

		// Setup physics environment
		mCollisionConfiguration	= new btDefaultCollisionConfiguration();
		mDispatcher				= new btCollisionDispatcher( mCollisionConfiguration );
		mBroadphase				= new btDbvtBroadphase();
		mSolver					= new btSequentialImpulseConstraintSolver();
	
		// Default dynamics
		mSoftBodyWorldInfo.air_density		= 1.2f;
		mSoftBodyWorldInfo.m_broadphase		= mBroadphase;
		mSoftBodyWorldInfo.m_dispatcher		= mDispatcher;
		mSoftBodyWorldInfo.m_gravity.setValue( 0.0f, -10.0f, 0.0f );
		mSoftBodyWorldInfo.water_density	= 0.0f;
		mSoftBodyWorldInfo.water_offset		= 0.0f;
		mSoftBodyWorldInfo.water_normal		= btVector3( 0.0f, 0.0f, 0.0f );
		mSoftBodyWorldInfo.m_sparsesdf.Initialize();

		// Build world
		mWorld = new btDiscreteDynamicsWorld( mDispatcher, mBroadphase, mSolver, mCollisionConfiguration );
		mWorld->setGravity( btVector3( 0.0f, -10.0f, 0.0f ) );
		mWorld->getDispatchInfo().m_enableSPU = true;

		mElapsedSeconds = getElapsedSeconds();
		mNumObjects = 0;

	}

	const DynamicsWorld& DynamicsWorld::operator=( const DynamicsWorld &rhs )
	{
		mBroadphase				= rhs.mBroadphase;
		mCamera					= rhs.mCamera;
		mCollisionConfiguration	= rhs.mCollisionConfiguration;
		mDispatcher				= rhs.mDispatcher;
		mElapsedSeconds			= rhs.mElapsedSeconds;
		mNumObjects				= rhs.mNumObjects;
		mSoftBodyWorldInfo		= rhs.mSoftBodyWorldInfo;
		mSolver					= rhs.mSolver;
		mWorld					= rhs.mWorld;
		mObjects.clear();
		mObjects.transfer( mObjects.begin(), rhs.mObjects );
		return *this;
	}

	DynamicsWorld::DynamicsWorld( const DynamicsWorld &rhs )
	{
		mBroadphase				= rhs.mBroadphase;
		mCamera					= rhs.mCamera;
		mCollisionConfiguration	= rhs.mCollisionConfiguration;
		mDispatcher				= rhs.mDispatcher;
		mElapsedSeconds			= rhs.mElapsedSeconds;
		mNumObjects				= rhs.mNumObjects;
		mSoftBodyWorldInfo		= rhs.mSoftBodyWorldInfo;
		mSolver					= rhs.mSolver;
		mWorld					= rhs.mWorld;
		mObjects.clear();
		mObjects.transfer( mObjects.begin(), rhs.mObjects );
	}

	DynamicsWorld::~DynamicsWorld()
	{
		if ( mBroadphase != 0 ) {
			delete mBroadphase;
		}
		if ( mCollisionConfiguration != 0 ) {
			delete mCollisionConfiguration;
		}
		if ( mDispatcher != 0 ) {
			delete mDispatcher;
		}
		if ( mWorld != 0 ) {
			delete mWorld;
		}
		if ( mSolver != 0 ) {
			delete mSolver;
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
			mWorld->addRigidBody( toBulletRigidBody( object ) );
		} else if ( object->isSoftBody() ) {
			( (btSoftRigidDynamicsWorld*)mWorld )->addSoftBody( toBulletSoftBody( object ) );
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
		constraint.mConstraint->m_setting.m_impulseClamp = clamping;
		constraint.mConstraint->m_setting.m_tau = tau;
	}

	CameraPersp& DynamicsWorld::getCamera()
	{
		return mCamera;
	}

	const CameraPersp& DynamicsWorld::getCamera() const
	{
		return mCamera;
	}

	bool DynamicsWorld::intersects( const Vec2f &pos, Constraint *constraint )
	{
		Ray ray				= mCamera.generateRay( pos.x, pos.y, getWindowAspectRatio() );
		btVector3 rayFrom	= toBulletVector3( mCamera.getEyePoint() );
		btVector3 rayTo		= toBulletVector3( ray.getOrigin() );

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

	}

	void DynamicsWorld::setCamera( const CameraPersp &camera )
	{
		mCamera = camera;
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////

	void DynamicsWorld::update( float frameRate )
	{

		// Check if object count has changed
		uint32_t numObjects = mWorld->getCollisionObjectArray().size();
		if ( mNumObjects != numObjects ) {

			// Nothing to do, bail
			if ( numObjects <= 0 ) {
				return;
			}

			// Activate all bodies
			for ( uint32_t i = 0; i < numObjects; i++ ) {
				btCollisionObject * collisionObject = mWorld->getCollisionObjectArray()[ i ];
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

		// Update object count
		mNumObjects = numObjects;

		// Update dynamics world
		mWorld->stepSimulation( 1.0f, 10, 1.0f / math<float>::max( 1.0f, frameRate ) );

	}

	/////////////////////////////////////////////////////////////////////////////////////////////////

	btRigidBody* DynamicsWorld::toBulletRigidBody( CollisionObject *object )
	{
		return (btRigidBody*)( object->getBulletBody() );
	}

	btRigidBody* DynamicsWorld::toBulletRigidBody( Iter pos )
	{
		return (btRigidBody*)( pos->getBulletBody() );
	}

	btSoftBody* DynamicsWorld::toBulletSoftBody( CollisionObject *object )
	{
		return (btSoftBody*)( object->getBulletBody() );
	}

	btSoftBody* DynamicsWorld::toBulletSoftBody( Iter pos )
	{
		return (btSoftBody*)( pos->getBulletBody() );
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////

	DynamicsWorldRef createWorld( const CameraPersp &camera )
	{
		return DynamicsWorldRef( new DynamicsWorld( camera ) );
	}

	CollisionObjectRef createRigidBox( const DynamicsWorldRef &world, const Vec3f &dimensions, float mass, const Vec3f &position, const Quatf &rotation )
	{
		return world->pushBack( new RigidBox( dimensions, mass, position, rotation ) );
	}

	CollisionObjectRef createRigidCone( const DynamicsWorldRef &world, float radius, float height, int32_t segments, float mass, 
		const Vec3f &position, const Quatf &rotation )
	{
		return world->pushBack( new RigidCone( radius, height, segments, mass, position, rotation ) );
	}

	CollisionObjectRef createRigidCylinder( const DynamicsWorldRef &world, const Vec3f &scale, int32_t segments, float mass, 
		const Vec3f &position, const Quatf &rotation )
	{
		return world->pushBack( new RigidCylinder( scale, segments, mass, position, rotation ) );
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

	CollisionObjectRef createRigidSphere( const DynamicsWorldRef &world, float radius, int32_t segments, float mass, const Vec3f &position, const Quatf &rotation )
	{
		return world->pushBack( new RigidSphere( radius, segments, mass, position, rotation ) );
	}

	CollisionObjectRef createRigidStaticPlane( const DynamicsWorldRef &world, const Vec3f &normal, float planeConstant, const Vec3f &position, const Quatf &rotation )
	{
		return world->pushBack( new RigidStaticPlane( normal, planeConstant, position, rotation ) );
	}

	CollisionObjectRef createRigidTerrain( const DynamicsWorldRef &world, const Channel32f &heightField, float minHeight, float maxHeight, 
		const Vec3f &scale, float mass, const Vec3f &position, const Quatf &rotation )
	{
		return world->pushBack( new RigidTerrain( heightField, minHeight, maxHeight, scale, mass, position, rotation ) );
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////

	btRigidBody* toBulletRigidBody( const CollisionObjectRef &object )
	{
		return (btRigidBody*)( object->getBulletBody() );
	}

	btSoftBody* toBulletSoftBody( const CollisionObjectRef &object )
	{
		return (btSoftBody*)( object->getBulletBody() );
	}

}
