// Include header
#include "DynamicsWorld.h"

#include "RigidObject.h"
#include "SoftObject.h"

namespace bullet {

	// Import namespaces
	using namespace ci;
	using namespace ci::app;
	using namespace std;

	/////////////////////////////////////////////////////////////////////////////////////////////////

	DynamicsWorldRef DynamicsWorld::getInstance()
	{
		static DynamicsWorldRef world;
		if ( !world ) {
			world = DynamicsWorldRef( new DynamicsWorld() );
		}
		return world;
	}

	// Constructor
	DynamicsWorld::DynamicsWorld()
	{

		// Setup physics environment
		mCollisionConfiguration = new btDefaultCollisionConfiguration();
		mDispatcher = new btCollisionDispatcher( mCollisionConfiguration );
		mBroadphase = new btDbvtBroadphase();
		mSolver = new btSequentialImpulseConstraintSolver();
	
		// Default dynamics
		mSoftBodyWorldInfo.air_density = 1.2f;
		mSoftBodyWorldInfo.m_broadphase = mBroadphase;
		mSoftBodyWorldInfo.m_dispatcher = mDispatcher;
		mSoftBodyWorldInfo.m_gravity.setValue( 0.0f, -10.0f, 0.0f );
		mSoftBodyWorldInfo.water_density = 0.0f;
		mSoftBodyWorldInfo.water_offset = 0.0f;
		mSoftBodyWorldInfo.water_normal = btVector3( 0.0f, 0.0f, 0.0f );
		mSoftBodyWorldInfo.m_sparsesdf.Initialize();

		// Build world
		mWorld = new btDiscreteDynamicsWorld( mDispatcher, mBroadphase, mSolver, mCollisionConfiguration );
		mWorld->setGravity( btVector3( 0.0f, -10.0f, 0.0f ) );
		mWorld->getDispatchInfo().m_enableSPU = true;

		// Set tracking properties
		mElapsedSeconds = getElapsedSeconds();
		mNumObjects = 0;

	}

	// Destructor
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
	
	Iter DynamicsWorld::end()
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

	void DynamicsWorld::pushBack( CollisionObjectBase *object )
	{
		mObjects.push_back( object );
		if ( object->isRigidBody() ) {
			mWorld->addRigidBody( toBulletRigidBody( object ) );
		} else if ( object->isSoftBody() ) {
			( (btSoftRigidDynamicsWorld*)mWorld )->addSoftBody( toBulletSoftBody( object ) );
		}
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////

	CollisionObject DynamicsWorld::createRigidBox( const ci::Vec3f &dimensions, float mass, const ci::Vec3f &position, 
		const ci::Quatf &rotation )
	{
		pushBack( new RigidBox( dimensions, mass, position, rotation ) );
		return &mObjects[ mObjects.size() - 1 ];
	}
	CollisionObject DynamicsWorld::createRigidCylinder( float topRadius, float bottomRadius, float height, int32_t segments, 
		float mass, const ci::Vec3f &position, const ci::Quatf &rotation )
	{
		pushBack( new RigidCylinder( topRadius, bottomRadius, height, segments, mass, position, rotation ) );
		return &mObjects[ mObjects.size() - 1 ];
	}
	CollisionObject DynamicsWorld::createRigidHull( const ci::TriMesh &mesh, const ci::Vec3f &scale, float mass, const ci::Vec3f &position, 
		const ci::Quatf &rotation )
	{
		pushBack( new RigidHull( mesh, scale, mass, position, rotation ) );
		return &mObjects[ mObjects.size() - 1 ];
	}
	CollisionObject DynamicsWorld::createRigidMesh( const ci::TriMesh &mesh, const ci::Vec3f &scale, float margin, 
		float mass, const ci::Vec3f &position, const ci::Quatf &rotation )
	{
		pushBack( new RigidMesh( mesh, scale, margin, mass, position, rotation ) );
		return &mObjects[ mObjects.size() - 1 ];
	}
	CollisionObject DynamicsWorld::createRigidSphere( float radius, int32_t segments, float mass, const ci::Vec3f &position, const ci::Quatf &rotation )
	{
		pushBack( new RigidSphere( radius, segments, mass, position, rotation ) );
		return &mObjects[ mObjects.size() - 1 ];
	}
	CollisionObject DynamicsWorld::createRigidTerrain( const ci::Surface32f &heightField, int32_t stickWidth, int32_t stickLength, float minHeight, 
		float maxHeight, int32_t upAxis, const ci::Vec3f &scale, float mass, const ci::Vec3f &position, const ci::Quatf &rotation )
	{
		pushBack( new RigidTerrain( heightField, stickWidth, stickLength, minHeight, maxHeight, upAxis, scale, mass, position, rotation ) );
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
	uint32_t DynamicsWorld::getNumObjects()
	{
		return mNumObjects;
	}
	CollisionObjectList& DynamicsWorld::getObjects()
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

	// Runs update logic
	void DynamicsWorld::update()
	{

		// Update objects
		double elapsedSeconds = getElapsedSeconds();
		double step = elapsedSeconds - mElapsedSeconds;
		mElapsedSeconds = elapsedSeconds;
		for ( Iter objectIt = begin(); objectIt != end(); ++objectIt ) {
			objectIt->update( step );
		}

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
					rigidBody->activate(true);
				} else {
					btSoftBody* softBody = btSoftBody::upcast( collisionObject );
					if ( softBody ) {
						softBody->activate(true);
					}
				}
			}

		}

		// Update object count
		mNumObjects = numObjects;

		// Update Bullet world
		mWorld->stepSimulation( 1.0f, 10, 1.0f / math<float>::max( 1.0f, getFrameRate() ) );

	}

	/////////////////////////////////////////////////////////////////////////////////////////////////

	btRigidBody* DynamicsWorld::toBulletRigidBody( CollisionObjectBase *object )
	{
		return (btRigidBody*)( object->getBulletBody() );
	}
	btRigidBody* DynamicsWorld::toBulletRigidBody( Iter pos )
	{
		return (btRigidBody*)( pos->getBulletBody() );
	}
	btSoftBody* DynamicsWorld::toBulletSoftBody( CollisionObjectBase *object )
	{
		return (btSoftBody*)( object->getBulletBody() );
	}
	btSoftBody* DynamicsWorld::toBulletSoftBody( Iter pos )
	{
		return (btSoftBody*)( pos->getBulletBody() );
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////

	Iter begin() 
	{
		return DynamicsWorld::getInstance()->begin();
	}
	Iter end()
	{
		return DynamicsWorld::getInstance()->end();
	}
	Iter erase( Iter pos )
	{
		return DynamicsWorld::getInstance()->erase( pos );
	}

	CollisionObject createRigidBox( const ci::Vec3f &dimensions, float mass, const ci::Vec3f &position, const ci::Quatf &rotation )
	{
		return DynamicsWorld::getInstance()->createRigidBox( dimensions, mass, position, rotation );
	}
	CollisionObject createRigidCylinder( float topRadius, float bottomRadius, float height, int32_t segments, float mass, 
		const ci::Vec3f &position, const ci::Quatf &rotation )
	{
		return DynamicsWorld::getInstance()->createRigidCylinder( topRadius, bottomRadius, height, segments, mass, position, rotation );
	}
	CollisionObject createRigidHull( const ci::TriMesh &mesh, const ci::Vec3f &scale, float mass, const ci::Vec3f &position, const ci::Quatf &rotation )
	{
		return DynamicsWorld::getInstance()->createRigidHull( mesh, scale, mass, position, rotation );
	}
	CollisionObject createRigidMesh( const ci::TriMesh &mesh, const ci::Vec3f &scale, float margin, float mass, 
		const ci::Vec3f &position, const ci::Quatf &rotation )
	{
		return DynamicsWorld::getInstance()->createRigidMesh( mesh, scale, margin, mass, position, rotation );
	}
	CollisionObject createRigidSphere( float radius, int32_t segments, float mass, const ci::Vec3f &position, const ci::Quatf &rotation )
	{
		return DynamicsWorld::getInstance()->createRigidSphere( radius, segments, mass, position, rotation );
	}
	CollisionObject createRigidTerrain( const ci::Surface32f &heightField, int32_t stickWidth, int32_t stickLength, float minHeight, 
		float maxHeight, int32_t upAxis, const ci::Vec3f &scale, float mass, const ci::Vec3f &position, const ci::Quatf &rotation )
	{
		return DynamicsWorld::getInstance()->createRigidTerrain( heightField, stickWidth, stickLength, minHeight, maxHeight, upAxis, scale, mass, position, rotation );
	}

	btBroadphaseInterface* getBroadphase()
	{
		return DynamicsWorld::getInstance()->getBroadphase();
	}
	btDefaultCollisionConfiguration* getCollisionConfiguration()
	{
		return DynamicsWorld::getInstance()->getCollisionConfiguration();
	}
	btCollisionDispatcher* getDispatcher()
	{
		return DynamicsWorld::getInstance()->getDispatcher();
	}
	btSoftBodyWorldInfo& getInfo()
	{
		return DynamicsWorld::getInstance()->getInfo();
	}
	uint32_t getNumObjects()
	{
		return DynamicsWorld::getInstance()->getNumObjects();
	}
	CollisionObjectList& getObjects()
	{
		return DynamicsWorld::getInstance()->getObjects();
	}
	btConstraintSolver* getSolver()
	{
		return DynamicsWorld::getInstance()->getSolver();
	}
	btDiscreteDynamicsWorld* getWorld()
	{
		return DynamicsWorld::getInstance()->getWorld();
	}

	void setInfo( const btSoftBodyWorldInfo &info )
	{
		DynamicsWorld::getInstance()->setInfo( info );
	}

	btRigidBody* toBulletRigidBody( const CollisionObject &object )
	{
		return (btRigidBody*)( object->getBulletBody() );
	}
	btSoftBody* toBulletSoftBody( const CollisionObject &object )
	{
		return (btSoftBody*)( object->getBulletBody() );
	}

	void update() 
	{
		DynamicsWorld::getInstance()->update();
	}

}
