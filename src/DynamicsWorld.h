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

namespace bullet
{

	// Reference alias
	typedef std::shared_ptr<class DynamicsWorld> DynamicsWorldRef;

	// Bullet physics world manager
	class DynamicsWorld
	{

	public:

		// Create pointer to instance
		static DynamicsWorldRef create();

		// Destructor
		~DynamicsWorld();

		// Draws everything
		void draw(bool wireframe = false);

		// Convenience methods for adding or removing object
		void erase(const CollisionObjectRefList::iterator & object);
		void erase(uint32_t index);
		bool eraseById(int32_t id);
		CollisionObjectRef insert(const CollisionObjectRef & object);

		// Handles contact
		void contactDown(const ci::Vec2i & position);
		void contactDrag(const ci::Vec2i & position);
			
		// Runs update logic
		void update();

		// Getters
		btDynamicsWorld * getWorld() { return mDynamicsWorld; }
		btBroadphaseInterface * getBroadphase() { return mBroadphase; }
		btCollisionDispatcher * getDispatcher() { return mDispatcher; }
		btSoftBodyWorldInfo getInfo() { return mSoftBodyWorldInfo; }
		CollisionObjectRef getObject(uint32_t index);
		CollisionObjectRef getObjectById(int32_t id);
		CollisionObjectRefList getObjects() { return mObjects; }
		btConstraintSolver * getSolver() { return mSolver; }
		btSoftBodyRigidBodyCollisionConfiguration * getCollisionConfiguration() { return mCollisionConfiguration; }

		// Setters
		void setInfo(const btSoftBodyWorldInfo & info);

	private:

		// Constructor
		DynamicsWorld();

		// Timeline
		double mElapsedSeconds;

		// Collision objects
		CollisionObjectRefList mObjects;

		// Bullet
		btBroadphaseInterface * mBroadphase;	
		btCollisionDispatcher * mDispatcher;
		btDynamicsWorld * mDynamicsWorld;
		btSoftBodyWorldInfo mSoftBodyWorldInfo;
		btConstraintSolver * mSolver;
		btSoftBodyRigidBodyCollisionConfiguration * mCollisionConfiguration;

		// Converts screen coordinates to Bullet world
		btVector3 screenToWorld(const ci::Vec2i & position);

	};

}
