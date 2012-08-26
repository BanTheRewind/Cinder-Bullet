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

#include "RigidBody.h"

#include "cinder/Utilities.h"

namespace bullet {
	using namespace ci;
	using namespace std;

	btRigidBody* RigidBody::create( btCollisionShape* shape, float mass, const Vec3f &position, const Quatf &rotation )
	{
		btVector3 inertia( 0.0f, 0.0f, 0.0f );
		if ( mass != 0.0f ) {
			shape->calculateLocalInertia( mass, inertia );
		}

		btQuaternion btRotation				= toBulletQuaternion( rotation );
		btVector3 btPosition				= toBulletVector3( position );
		btTransform worldTransform( btRotation, btPosition );
		btDefaultMotionState* motionState	= new btDefaultMotionState( worldTransform );
		btRigidBody::btRigidBodyConstructionInfo info( mass, motionState, shape, inertia );

		btRigidBody* body = new btRigidBody( info );
		return body;
	}

	btRigidBody* RigidBody::createBox( const Vec3f &size, float mass, const Vec3f &position, const Quatf &rotation )
	{
		btVector3 halfSize		= toBulletVector3( size ) * 0.5f;
		btCollisionShape* shape = new btBoxShape( halfSize );
		btRigidBody* body = create( shape, mass, position, rotation );
		return body;
	}

	btRigidBody* RigidBody::createCone( float radius, float height, float mass, const Vec3f &position, const Quatf &rotation )
	{
		btCollisionShape* shape = new btConeShape( radius, height );
		btRigidBody* body		= create( shape, mass, position, rotation );
		return body;
	}

	btRigidBody* RigidBody::createCylinder( const Vec3f &scale, float mass, const Vec3f &position, const Quatf &rotation )
	{
		btVector3 size			= toBulletVector3( scale );
		btCollisionShape* shape = new btCylinderShape( size );
		btRigidBody* body		= create( shape, mass, position, rotation );
		return body;
	}

	btRigidBody* RigidBody::createHull( btConvexHullShape* shape, float mass, const Vec3f &position, const Quatf &rotation )
	{
		btRigidBody* body = create( shape, mass, position, rotation );
		return body;
	}

	btRigidBody* RigidBody::createMesh( btBvhTriangleMeshShape* shape, float mass, const Vec3f &position, const Quatf &rotation )
	{
		btRigidBody* body = create( shape, mass, position, rotation );
		return body;
	}

	btRigidBody* RigidBody::createSphere( float radius, float mass, const Vec3f &position, const Quatf &rotation )
	{
		btCollisionShape* shape = new btSphereShape( ( btScalar ) radius );
		btRigidBody* body		= create( shape, mass, position, rotation );
		return body;
	}

	btRigidBody* RigidBody::createTerrain( btHeightfieldTerrainShape* shape, float mass, const ci::Vec3f &position, const ci::Quatf &rotation )
	{
		btRigidBody* body = create( shape, mass, position, rotation );
		return body;
	}

	RigidBox::RigidBox( const Vec3f &dimensions, float mass, const Vec3f &position, const Quatf &rotation ) 
		: CollisionObject( position, rotation )
	{
		mPrimitiveType		= PRIMITIVE_BOX;
		mRigidBody			= createBox( dimensions, mass, position, rotation );
		mScale				= dimensions;
	}

	RigidCone::RigidCone( float radius, float height, float mass, const Vec3f &position, const Quatf &rotation )
		: CollisionObject( position, rotation )
	{
		mPrimitiveType		= PRIMITIVE_CONE;
		mRigidBody			= createCone( radius, height, mass, position, rotation );
		mScale				= Vec3f( radius, height, radius );
	}

	RigidCylinder::RigidCylinder( const Vec3f &scale, float mass, const Vec3f &position, const Quatf &rotation )
		: CollisionObject( position, rotation )
	{
		mPrimitiveType		= PRIMITIVE_CYLINDER;
		mRigidBody			= createCylinder( scale, mass, position, rotation );
		mScale				= scale;
	}

	RigidHull::RigidHull( const TriMesh &mesh, const Vec3f &scale, float mass, const Vec3f &position, const Quatf &rotation )
		: CollisionObject( position, rotation )
	{
		mScale		= scale;

		btConvexHullShape* shape = createConvexHullShape( mesh.getVertices(), scale );
		mRigidBody = createHull( shape, mass, position, rotation );

		mIndices	= mesh.getIndices();
		mNormals	= mesh.getNormals();
		mPositions	= mesh.getVertices();
		mTexCoords	= mesh.getTexCoords();
	}

	RigidMesh::RigidMesh( const TriMesh &mesh, const Vec3f &scale, float margin, float mass, const Vec3f &position, const Quatf &rotation )
		: CollisionObject( position, rotation )
	{
		mScale		= scale;

		Vec3f halfScale = scale * 0.5f;
		btBvhTriangleMeshShape* shape = createConcaveMeshShape( mesh.getVertices(), mesh.getIndices(), scale, margin );
		mRigidBody = createMesh( shape, mass, position, rotation );

		mIndices	= mesh.getIndices();
		mNormals	= mesh.getNormals();
		mPositions	= mesh.getVertices();
		mTexCoords	= mesh.getTexCoords();
	}

	RigidSphere::RigidSphere( float radius, float mass, const Vec3f &position, const Quatf &rotation ) 
		: CollisionObject( position, rotation )
	{
		mPrimitiveType		= PRIMITIVE_SPHERE;
		mRigidBody			= createSphere( radius, mass, position, rotation );
		mScale				= Vec3f::one() * radius;
	}

	RigidTerrain::RigidTerrain( const Channel32f &heightField, float minHeight, float maxHeight, const Vec3f &scale, float mass, 
		const Vec3f &position, const Quatf &rotation )
		: CollisionObject( position, rotation )
	{
		mChannel	= heightField;
		mScale		= scale;

		btHeightfieldTerrainShape* shape = createHeightfieldTerrainShape( mChannel, minHeight, maxHeight, scale );
		mRigidBody = createTerrain( shape, mass, position, rotation );

		int32_t height	= mChannel.getHeight();
		int32_t width	= mChannel.getWidth();

		for ( int32_t y = 0; y < height; y++ ) {
			for ( int32_t x = 0; x < width; x++ ) {
				mTexCoords.push_back( Vec2f( (float)x / (float)width, (float)y / (float)height ) );

				int32_t xn = x + 1 >= width ? 0 : 1;
				int32_t yn = y + 1 >= height ? 0 : 1;
				mIndices.push_back( x + height * y );
				mIndices.push_back( ( x + xn ) + height * y);
				mIndices.push_back( ( x + xn ) + height * ( y + yn ) );
				mIndices.push_back( x + height * ( y + yn ) );
				mIndices.push_back( ( x + xn ) + height * ( y + yn ) );
				mIndices.push_back( x + height * y );
			}
		}

		readChannelData();
	}

	void RigidTerrain::readChannelData()
	{
		mNormals.clear();
		mPositions.clear();

		int32_t height		= mChannel.getHeight();
		int32_t width		= mChannel.getWidth();
		float halfHeight	= (float)height * 0.5f;
		float halfWidth		= (float)width * 0.5f;
		
		for ( int32_t y = 0; y < height; y++ ) {
			for ( int32_t x = 0; x < width; x++ ) {
				float value = mChannel.getValue( Vec2i( x, y ) );

				Vec3f position( (float)x - halfWidth, value, (float)y - halfHeight );
				mPositions.push_back( position );

				mNormals.push_back( Vec3f::zero() );
			}
		}

		for ( int32_t y = 0; y < height - 1; y++ ) {
			for ( int32_t x = 0; x < width - 1; x++ ) {
				Vec3f vert0 = mPositions[ mIndices[ ( x + height * y ) * 6 ] ];
				Vec3f vert1 = mPositions[ mIndices[ ( ( x + 1 ) + height * y ) * 6 ] ];
				Vec3f vert2 = mPositions[ mIndices[ ( ( x + 1 ) + height * ( y + 1 ) ) * 6 ] ];
				mNormals[ x + height * y ] = Vec3f( ( vert1 - vert0 ).cross( vert1 - vert2 ).normalized() );
			}
		}
	}
}
