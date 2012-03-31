// Include header
#include "DynamicsWorld.h"

#include "RigidObject.h"
#include "SoftObject.h"

namespace bullet {

	// Import namespaces
	using namespace ci;
	using namespace ci::app;
	using namespace std;

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
		mCollisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();
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
		mWorld = new btSoftRigidDynamicsWorld( mDispatcher, mBroadphase, mSolver, mCollisionConfiguration );
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
		return mObjects.erase( pos );
	}

	CollisionObject DynamicsWorld::createRigidBox( const ci::Vec3f &dimensions, const ci::Vec3f &position, const ci::Quatf &rotation )
	{
		mObjects.push_back( new RigidBox( mWorld, dimensions, position, rotation ) );
		return mObjects.back();
	}
	CollisionObject DynamicsWorld::createRigidCylinder( float topRadius, float bottomRadius, float height, int32_t segments, 
		const ci::Vec3f &position, const ci::Quatf &rotation )
	{
		mObjects.push_back( new RigidCylinder( mWorld, topRadius, bottomRadius, height, segments, position, rotation ) );
		return mObjects.back();
	}
	CollisionObject DynamicsWorld::createRigidHull( const ci::TriMesh &mesh, const ci::Vec3f &scale, const ci::Vec3f &position, const ci::Quatf &rotation )
	{
		mObjects.push_back( new RigidHull( mWorld, mesh, scale, position, rotation ) );
		return mObjects.back();
	}
	CollisionObject DynamicsWorld::createRigidMesh( const ci::TriMesh &mesh, const ci::Vec3f &scale, float margin, 
		const ci::Vec3f &position, const ci::Quatf &rotation )
	{
		mObjects.push_back( new RigidMesh( mWorld, mesh, scale, margin, position, rotation ) );
		return mObjects.back();
	}
	CollisionObject DynamicsWorld::createRigidSphere( float radius, int32_t segments, const ci::Vec3f &position, const ci::Quatf &rotation )
	{
		mObjects.push_back( new RigidSphere( mWorld, radius, segments, position, rotation ) );
		return mObjects.back();
	}
	CollisionObject DynamicsWorld::createRigidTerrain( const ci::Surface32f &heightField, int32_t stickWidth, int32_t stickLength, float minHeight, 
		float maxHeight, int32_t upAxis, const ci::Vec3f &scale, const ci::Vec3f &position, const ci::Quatf &rotation )
	{
		mObjects.push_back( new RigidTerrain( mWorld, heightField, stickWidth, stickLength, minHeight, maxHeight, upAxis, scale, position, rotation ) );
		return mObjects.back();
	}

	btBroadphaseInterface* DynamicsWorld::getBroadphase() 
	{ 
		return mBroadphase; 
	}
	btSoftBodyRigidBodyCollisionConfiguration* DynamicsWorld::getCollisionConfiguration() 
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
	btDynamicsWorld* DynamicsWorld::getWorld() 
	{ 
		return mWorld; 
	}

	// Set soft body info
	void DynamicsWorld::setInfo( const btSoftBodyWorldInfo &info )
	{
		mSoftBodyWorldInfo = info;
	}

	// Runs update logic
	void DynamicsWorld::update()
	{

		// Get time since last frame, update
		double elapsedSeconds = getElapsedSeconds();
		double step = elapsedSeconds - mElapsedSeconds;
		mElapsedSeconds = elapsedSeconds;

		// Update Bullet world
		mWorld->stepSimulation( 1.0f, 10, 1.0f / math<float>::max( 1.0f, getFrameRate() ) );

		// Check if object count has changed
		uint32_t numObjects = mWorld->getNumCollisionObjects();
		if ( mNumObjects != numObjects ) {

			// Nothing to do, bail
			if ( numObjects <= 0 ) {
				return;
			}

			// Activate all bodies
			for ( uint32_t i = 0; i < numObjects; i++ ) {
				btCollisionObject * collisionObject = mWorld->getCollisionObjectArray().at( i );
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

	}

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

	CollisionObject createRigidBox( const ci::Vec3f &dimensions, const ci::Vec3f &position, const ci::Quatf &rotation )
	{
		return DynamicsWorld::getInstance()->createRigidBox( dimensions, position, rotation );
	}
	CollisionObject createRigidCylinder( float topRadius, float bottomRadius, float height, int32_t segments, 
		const ci::Vec3f &position, const ci::Quatf &rotation )
	{
		return DynamicsWorld::getInstance()->createRigidCylinder( topRadius, bottomRadius, height, segments, position, rotation );
	}
	CollisionObject createRigidHull( const ci::TriMesh &mesh, const ci::Vec3f &scale, const ci::Vec3f &position, const ci::Quatf &rotation )
	{
		return DynamicsWorld::getInstance()->createRigidHull( mesh, scale, position, rotation );
	}
	CollisionObject createRigidMesh( const ci::TriMesh &mesh, const ci::Vec3f &scale, float margin, 
		const ci::Vec3f &position, const ci::Quatf &rotation )
	{
		return DynamicsWorld::getInstance()->createRigidMesh( mesh, scale, margin, position, rotation );
	}
	CollisionObject createRigidSphere( float radius, int32_t segments, const ci::Vec3f &position, const ci::Quatf &rotation )
	{
		return DynamicsWorld::getInstance()->createRigidSphere( radius, segments, position, rotation );
	}
	CollisionObject createRigidTerrain( const ci::Surface32f &heightField, int32_t stickWidth, int32_t stickLength, float minHeight, 
		float maxHeight, int32_t upAxis, const ci::Vec3f &scale, const ci::Vec3f &position, const ci::Quatf &rotation )
	{
		return DynamicsWorld::getInstance()->createRigidTerrain( heightField, stickWidth, stickLength, minHeight, maxHeight, upAxis, scale, position, rotation );
	}

	btBroadphaseInterface* getBroadphase()
	{
		return DynamicsWorld::getInstance()->getBroadphase();
	}
	btSoftBodyRigidBodyCollisionConfiguration* getCollisionConfiguration()
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
	btDynamicsWorld* getWorld()
	{
		return DynamicsWorld::getInstance()->getWorld();
	}

	void setInfo( const btSoftBodyWorldInfo &info )
	{
		DynamicsWorld::getInstance()->setInfo( info );
	}

	void update() 
	{
		DynamicsWorld::getInstance()->update();
	}

}
