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

#include "CollisionObject.h"

namespace bullet
{

	using namespace ci;
	using namespace std;

	CollisionObjectBase::CollisionObjectBase( const Vec3f &position, const Quatf &rotation ) 
	{

		// Initialize VBO layout
		mVboLayout.setStaticIndices();
		mVboLayout.setStaticNormals();
		mVboLayout.setStaticPositions();

		// Define properties
		mAge = 0.0;
		mPosition = position;
		mRigidBody = 0;
		mSoftBody = 0;
		mTransformMatrix.setToIdentity();

	}

	CollisionObjectBase::~CollisionObjectBase() 
	{
		if (mSoftBody != 0 ) {
			delete mSoftBody;
		}
		if ( mRigidBody != 0 ) {
			if ( mRigidBody->getMotionState() ) {
				delete mRigidBody->getMotionState();
			}
			delete mRigidBody;
		}
	}

	void CollisionObjectBase::clearVboData()
	{
		mVboIndices.clear();
		mVboNormals.clear();
		mVboPositions.clear();
	}

	double CollisionObjectBase::getAge() 
	{ 
		return mAge; 
	}
	double CollisionObjectBase::getAge() const 
	{ 
		return mAge; 
	}
	
	btCollisionObject* CollisionObjectBase::getBulletBody() 
	{ 
		if ( mSoftBody != 0 ) {
			return (btCollisionObject*)mSoftBody;
		} else if ( mRigidBody != 0 ) {
			return (btCollisionObject*)mRigidBody;
		}
		return 0;
	}
	btCollisionObject* CollisionObjectBase::getBulletBody() const
	{ 
		if ( mSoftBody != 0 ) {
			return (btCollisionObject*)mSoftBody;
		} else if ( mRigidBody != 0 ) {
			return (btCollisionObject*)mRigidBody;
		}
		return 0;
	}

	ci::Vec3f& CollisionObjectBase::getPosition() 
	{ 
		return mPosition; 
	}
	const ci::Vec3f& CollisionObjectBase::getPosition() const 
	{ 
		return mPosition; 
	}

	ci::Matrix44f& CollisionObjectBase::getTransformMatrix() 
	{ 
		return mTransformMatrix;
	}
	const ci::Matrix44f& CollisionObjectBase::getTransformMatrix() const 
	{ 
		return mTransformMatrix;
	}

	ci::gl::VboMesh& CollisionObjectBase::getVboMesh() 
	{ 
		return mVboMesh; 
	}
	const ci::gl::VboMesh& CollisionObjectBase::getVboMesh() const 
	{ 
		return mVboMesh; 
	}

	bool CollisionObjectBase::isRigidBody()
	{
		return mRigidBody != 0;
	}
	bool CollisionObjectBase::isRigidBody() const
	{
		return mRigidBody != 0;
	}
	bool CollisionObjectBase::isSoftBody()
	{
		return mSoftBody != 0;
	}
	bool CollisionObjectBase::isSoftBody() const
	{
		return mSoftBody != 0;
	}

	void CollisionObjectBase::setVboData( GLenum primitiveType )
	{
		mVboMesh = gl::VboMesh( mVboPositions.size(), mVboIndices.size(), mVboLayout, primitiveType );
		mVboMesh.bufferIndices( mVboIndices );
		mVboMesh.bufferNormals( mVboNormals );
		mVboMesh.bufferPositions( mVboPositions );
	}

	void CollisionObjectBase::update( double step ) 
	{
		mAge += step;
		if ( mSoftBody != 0 ) {
			mPosition = Utilities::fromBulletVector3( mSoftBody->m_bounds[ 0 ].lerp( mSoftBody->m_bounds[ 1 ], 0.5f ) );
			mTransformMatrix = Utilities::getWorldTransform( mSoftBody ); 
		} else {
			mPosition = Utilities::fromBulletVector3( mRigidBody->getCenterOfMassPosition() );
			mTransformMatrix = Utilities::getWorldTransform( mRigidBody );
		}
	}

}
