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
		mScale = Vec3f::one();
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
			worldTransform = getTransformMatrix( mSoftBody ); 
		} else if ( mRigidBody != 0 ) {
			worldTransform = getTransformMatrix( mRigidBody );
		}
		worldTransform.scale( mScale );
		return worldTransform;
	}
	ci::Matrix44f CollisionObject::getTransformMatrix() const 
	{ 
		Matrix44f worldTransform;
		if ( mSoftBody != 0 ) {
			worldTransform = getTransformMatrix( mSoftBody ); 
		} else if ( mRigidBody != 0 ) {
			worldTransform = getTransformMatrix( mRigidBody );
		}
		worldTransform.scale( mScale );
		return worldTransform;
	}

	ci::gl::VboMesh& CollisionObject::getVboMesh() 
	{ 
		return *mVboMesh; 
	}
	const ci::gl::VboMesh& CollisionObject::getVboMesh() const 
	{ 
		return *mVboMesh; 
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

	/////////////////////////////////////////////////////////////////////////////////////////////////

	// Convert Bullet matrix to Cinder matrix for rigid bodies
	Matrix44f CollisionObject::getTransformMatrix( const btRigidBody* body )
	{
		btTransform trans;
		body->getMotionState()->getWorldTransform( trans );
		Matrix44f matrix;
		trans.getOpenGLMatrix( matrix.m );
		return matrix;
	}

	// Convert Bullet matrix to Cinder matrix for soft bodies
	Matrix44f CollisionObject::getTransformMatrix( const btSoftBody* body )
	{
		btTransform trans = body->getWorldTransform();
		Matrix44f matrix;
		trans.getOpenGLMatrix( matrix.m );
		return matrix;
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////

	// Create terrain from color channel
	btHeightfieldTerrainShape* CollisionObject::createHeightfieldTerrainShape( const Channel32f &heightField, float minHeight, float maxHeight, const Vec3f &scale )
	{

		// Get stick size from image dimensions
		int32_t length = heightField.getHeight(); 
		int32_t width = heightField.getWidth();

        // Create height field shape from channel data
		float heightScale = math<float>::abs( minHeight ) + math<float>::abs( maxHeight );
		btHeightfieldTerrainShape* shape = new btHeightfieldTerrainShape( width, length, heightField.getData(), heightScale, minHeight, maxHeight, 1, PHY_FLOAT, false );

		// Scale and return shape
		shape->setLocalScaling( toBulletVector3( scale ) );
		return shape;

	}

	// Creates a concave Bullet mesh from a list of vertices and indices
	btBvhTriangleMeshShape* CollisionObject::createConcaveMeshShape( const vector<Vec3f>& vertices, const vector<uint32_t>& indices, const Vec3f& scale, float margin )
	{

		// Create Bullet mesh
		btTriangleMesh* triMesh = new btTriangleMesh( true, false );

		// Add triangles
		uint32_t numTriangles = indices.size() / 3;
		for ( uint32_t i = 0; i < numTriangles; i += 3 )
			triMesh->addTriangle( 
			toBulletVector3( vertices.at( indices.at( i + 0 ) ) ), 
			toBulletVector3( vertices.at( indices.at( i + 1 ) ) ), 
			toBulletVector3( vertices.at( indices.at( i + 2 ) ) ), 
			true
			 );

		// Create mesh shape
		btBvhTriangleMeshShape* shape = new btBvhTriangleMeshShape( triMesh, true );
		shape->buildOptimizedBvh();

		// Scale and return shape
		btVector3 localScaling = toBulletVector3( scale );
		shape->setLocalScaling( localScaling );
		shape->setMargin( margin );
		return shape;

	}

	// Creates a convex Bullet hull from a list of vertices
	btConvexHullShape* CollisionObject::createConvexHullShape( const vector<Vec3f>& vertices, const Vec3f& scale )
	{

		// Create hull
		btConvexHullShape* shape = new btConvexHullShape();

		// Add points
		for ( uint32_t i = 0; i < vertices.size(); i++ ) {
			shape->addPoint( toBulletVector3( vertices.at( i ) ) );
		}

		// Scale and return shape
		btVector3 localScaling = toBulletVector3( scale );
		shape->setLocalScaling( localScaling );
		return shape;

	}

}
