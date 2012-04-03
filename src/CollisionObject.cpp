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

	CollisionObject::CollisionObject( const Vec3f &position, const Quatf &rotation ) 
	{
		mRigidBody = 0;
		mSoftBody = 0;
	}

	CollisionObject::~CollisionObject() 
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
	
	btCollisionObject* CollisionObject::getBulletBody() 
	{ 
		if ( mSoftBody != 0 ) {
			return (btCollisionObject*)mSoftBody;
		} else if ( mRigidBody != 0 ) {
			return (btCollisionObject*)mRigidBody;
		}
		return 0;
	}
	btCollisionObject* CollisionObject::getBulletBody() const
	{ 
		if ( mSoftBody != 0 ) {
			return (btCollisionObject*)mSoftBody;
		} else if ( mRigidBody != 0 ) {
			return (btCollisionObject*)mRigidBody;
		}
		return 0;
	}

	ci::Vec3f CollisionObject::getPosition() 
	{ 
		Vec3f position;
		if ( mSoftBody != 0 ) {
			position = fromBulletVector3( mSoftBody->m_bounds[ 0 ].lerp( mSoftBody->m_bounds[ 1 ], 0.5f ) );
		} else if ( mRigidBody != 0 ) {
			position = fromBulletVector3( mRigidBody->getCenterOfMassPosition() );
		}
		return position;
	}
	ci::Vec3f CollisionObject::getPosition() const 
	{ 
		Vec3f position;
		if ( mSoftBody != 0 ) {
			position = fromBulletVector3( mSoftBody->m_bounds[ 0 ].lerp( mSoftBody->m_bounds[ 1 ], 0.5f ) );
		} else if ( mRigidBody != 0 ) {
			position = fromBulletVector3( mRigidBody->getCenterOfMassPosition() );
		}
		return position;
	}

	ci::Matrix44f CollisionObject::getTransformMatrix() 
	{ 
		Matrix44f worldTransform;
		if ( mSoftBody != 0 ) {
			worldTransform = Utilities::getWorldTransform( mSoftBody ); 
		} else if ( mRigidBody != 0 ) {
			worldTransform = Utilities::getWorldTransform( mRigidBody );
		}
		return worldTransform;
	}
	ci::Matrix44f CollisionObject::getTransformMatrix() const 
	{ 
		Matrix44f worldTransform;
		if ( mSoftBody != 0 ) {
			worldTransform = Utilities::getWorldTransform( mSoftBody ); 
		} else if ( mRigidBody != 0 ) {
			worldTransform = Utilities::getWorldTransform( mRigidBody );
		}
		return worldTransform;
	}

	ci::gl::VboMesh& CollisionObject::getVboMesh() 
	{ 
		return mVboMesh; 
	}
	const ci::gl::VboMesh& CollisionObject::getVboMesh() const 
	{ 
		return mVboMesh; 
	}

	bool CollisionObject::isRigidBody()
	{
		return mRigidBody != 0;
	}
	bool CollisionObject::isRigidBody() const
	{
		return mRigidBody != 0;
	}
	bool CollisionObject::isSoftBody()
	{
		return mSoftBody != 0;
	}
	bool CollisionObject::isSoftBody() const
	{
		return mSoftBody != 0;
	}

	void CollisionObject::setVboData( const vector<uint32_t> &indices, const vector<Vec3f> &positions, 
		const vector<Vec3f> &normals, const vector<Vec2f> &texCoords, GLenum primitiveType )
	{

		ci::gl::VboMesh::Layout layout;
		if ( indices.size() > 0 ) {
			layout.setStaticIndices();
		}
		if ( normals.size() > 0 ) {
			layout.setStaticNormals();
		}
		if ( positions.size() > 0 ) {
			layout.setStaticPositions();
		}
		if ( texCoords.size() > 0 ) {
			layout.setStaticTexCoords2d();
		}

		mVboMesh = gl::VboMesh( positions.size(), indices.size(), layout, primitiveType );
		if ( indices.size() > 0 ) {
			mVboMesh.bufferIndices( indices );
		}
		if ( normals.size() > 0 ) {
			mVboMesh.bufferNormals( normals );
		}
		if ( positions.size() > 0 ) {
			mVboMesh.bufferPositions( positions );
		}
		if ( texCoords.size() > 0 ) {
			mVboMesh.bufferTexCoords2d( 0, texCoords );
		}

	}

}
