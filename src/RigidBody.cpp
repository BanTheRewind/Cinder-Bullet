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

// Include header
#include "RigidBody.h"

#include "cinder/Utilities.h"

namespace bullet
{

	// Import namespaces
	using namespace ci;
	using namespace std;

	// Creates rigid body from any collision shape
	btRigidBody* RigidBody::create( btCollisionShape* shape, float mass, const Vec3f &position, const Quatf &rotation )
	{

		// Calculate inertia
		btVector3 inertia( 0.0f, 0.0f, 0.0f );
		if ( mass != 0.0f ) {
			shape->calculateLocalInertia( mass, inertia );
		}

		// Define bullet body
		btQuaternion btRotation				= toBulletQuaternion( rotation );
		btVector3 btPosition				= toBulletVector3( position );
		btTransform worldTransform( btRotation, btPosition );
		btDefaultMotionState* motionState	= new btDefaultMotionState( worldTransform );
		btRigidBody::btRigidBodyConstructionInfo info( mass, motionState, shape, inertia );

		// Create and return body
		btRigidBody* body = new btRigidBody( info );
		return body;

	}

	// Creates a rigid box
	btRigidBody* RigidBody::createBox( const Vec3f &size, float mass, const Vec3f &position, const Quatf &rotation )
	{

		// Create Bullet box
		btVector3 halfSize = toBulletVector3( size ) * 0.5f;
		btCollisionShape* shape = new btBoxShape( halfSize );

		// Create and return rigid body
		btRigidBody* body = create( shape, mass, position, rotation );
		return body;

	}

	// Creates rigid cone
	btRigidBody* RigidBody::createCone( float radius, float height, int32_t segments, float mass, const Vec3f &position, const Quatf &rotation )
	{

		// Create cylinder
		btCollisionShape* shape = new btConeShape( radius, height );

		// Create and return rigid body
		btRigidBody* body = create( shape, mass, position, rotation );
		return body;

	}

	// Creates rigid cylinder
	btRigidBody* RigidBody::createCylinder( const Vec3f &scale, int32_t segments, float mass, const Vec3f &position, const Quatf &rotation )
	{

		// Create cylinder
		btVector3 size			= toBulletVector3( scale );
		btCollisionShape* shape = new btCylinderShape( size );

		// Create and return rigid body
		btRigidBody* body = create( shape, mass, position, rotation );
		return body;

	}

	// Create rigid body from convex hull shape
	btRigidBody* RigidBody::createHull( btConvexHullShape* shape, float mass, const Vec3f &position, const Quatf &rotation )
	{

		// Create and return rigid body
		btRigidBody* body = create( shape, mass, position, rotation );
		return body;

	}

	// Create rigid body from triangle mesh
	btRigidBody* RigidBody::createMesh( btBvhTriangleMeshShape* shape, float mass, const Vec3f &position, const Quatf &rotation )
	{

		// Create and return rigid body
		btRigidBody* body = create( shape, mass, position, rotation );
		return body;

	}

	// Creates a rigid sphere
	btRigidBody* RigidBody::createSphere( float radius, float mass, const Vec3f &position, const Quatf &rotation )
	{

		// Create Bullet sphere
		btCollisionShape* shape = new btSphereShape( ( btScalar ) radius );

		// Create and return rigid body
		btRigidBody* body = create( shape, mass, position, rotation );
		return body;

	}

	// Creates a static plane
	btRigidBody* RigidBody::createStaticPlane( const Vec3f &normal, float planeConstant, const Vec3f &position, const Quatf &rotation )
	{

		// Create Bullet floor
		btCollisionShape* shape = new btStaticPlaneShape( toBulletVector3( normal ), planeConstant );

		// Create and return rigid body
		btRigidBody* body = create( shape, 0.0f, position, rotation );
		return body;

	}

	// Creates terrain
	btRigidBody* RigidBody::createTerrain( btHeightfieldTerrainShape* shape, float mass, const ci::Vec3f &position, const ci::Quatf &rotation )
	{

		// Create and return rigid body
		btRigidBody* body = create( shape, mass, position, rotation );
		return body;

	}

	RigidBox::RigidBox( const Vec3f &dimensions, float mass, const Vec3f &position, const Quatf &rotation ) 
		: CollisionObject( position, rotation )
	{

		// Create body
		mRigidBody = createBox( dimensions, mass, position, rotation );

		// Set scale
		mScale = dimensions;

		// Create VBO
		mVboMesh = VboMeshManager::create( VboMeshManager::PRIMITIVE_BOX );

	}

	RigidCone::RigidCone( float radius, float height, int32_t segments, float mass, const Vec3f &position, const Quatf &rotation )
		: CollisionObject( position, rotation )
	{

		// Create body
		mRigidBody = createCone( radius, height, segments, mass, position, rotation );

		// Set scale
		mScale = Vec3f( radius, height, radius );

		// Create VBO
		mVboMesh = VboMeshManager::create( VboMeshManager::PRIMITIVE_CONE, segments );

	}

	RigidCylinder::RigidCylinder( const Vec3f &scale, int32_t segments, float mass, const Vec3f &position, const Quatf &rotation )
		: CollisionObject( position, rotation )
	{

		// Create body
		mRigidBody = createCylinder( scale, segments, mass, position, rotation );

		// Set scale
		mScale = scale;

		// Create VBO
		mVboMesh = VboMeshManager::create( VboMeshManager::PRIMITIVE_CYLINDER, segments );

	}

	// Convex hull mesh
	RigidHull::RigidHull( const TriMesh &mesh, const Vec3f &scale, float mass, const Vec3f &position, const Quatf &rotation )
		: CollisionObject( position, rotation )
	{

		// Set scale
		mScale = scale;

		// Define body
		btConvexHullShape* shape = createConvexHullShape( mesh.getVertices(), scale );
		mRigidBody = createHull( shape, mass, position, rotation );

		// Set data from TriMesh
		mIndices = mesh.getIndices();
		mNormals = mesh.getNormals();
		mPositions = mesh.getVertices();
		mTexCoords = mesh.getTexCoords();

		// Set VBO data
		mVboMesh = VboMeshManager::create( mIndices, mPositions, mNormals, mTexCoords );

	}

	// Concave mesh
	RigidMesh::RigidMesh( const TriMesh &mesh, const Vec3f &scale, float margin, float mass, const Vec3f &position, const Quatf &rotation )
		: CollisionObject( position, rotation )
	{

		// Set scale
		mScale = scale;

		// Define body
		Vec3f halfScale = scale * 0.5f;
		btBvhTriangleMeshShape* shape = createConcaveMeshShape( mesh.getVertices(), mesh.getIndices(), scale, margin );
		mRigidBody = createMesh( shape, mass, position, rotation );

		// Set data from TriMesh
		mIndices	= mesh.getIndices();
		mNormals	= mesh.getNormals();
		mPositions	= mesh.getVertices();
		mTexCoords	= mesh.getTexCoords();

		// Set VBO data
		mVboMesh = VboMeshManager::create( mIndices, mPositions, mNormals, mTexCoords );
		
	}

	// Sphere
	RigidSphere::RigidSphere( float radius, int32_t segments, float mass, const Vec3f &position, const Quatf &rotation ) 
		: CollisionObject( position, rotation )
	{

		// Create body
		mRigidBody = createSphere( radius, mass, position, rotation );

		// Set scale
		mScale = Vec3f::one() * radius;

		// Create VBO
		mVboMesh = VboMeshManager::create( VboMeshManager::PRIMITIVE_SPHERE, segments );

	}

	// Static plane
	RigidStaticPlane::RigidStaticPlane( const Vec3f &normal, float planeConstant, const Vec3f &position, const Quatf &rotation ) 
		: CollisionObject( position, rotation )
	{

		// Create body
		mRigidBody = createStaticPlane( normal, planeConstant, position, rotation );

		// Set scale
		mScale = Vec3f::one();

	}

	// Terrain from Surface data
	RigidTerrain::RigidTerrain( const Channel32f &heightField, float minHeight, float maxHeight, const Vec3f &scale, float mass, 
		const Vec3f &position, const Quatf &rotation )
		: CollisionObject( position, rotation )
	{

		// Copy channel
		mChannel = heightField;

		// Set scale
		mScale = scale;

		// Create body
		btHeightfieldTerrainShape* shape = createHeightfieldTerrainShape( mChannel, minHeight, maxHeight, scale );
		mRigidBody = createTerrain( shape, mass, position, rotation );

		// Get image dimensions
		int32_t height	= mChannel.getHeight();
		int32_t width	= mChannel.getWidth();

		// Iterate through dimensions to set indices and texture coordinates
		for ( int32_t y = 0; y < height; y++ ) {
			for ( int32_t x = 0; x < width; x++ ) {

				// Add texture coordinate
				mTexCoords.push_back( Vec2f( (float)x / (float)width, (float)y / (float)height ) );

				// Add indices for this quad
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

		// Define positions, normals from channel
		readChannelData();

		// Set VBO
		mVboMesh = VboMeshManager::create( mIndices, mPositions, mNormals, mTexCoords );

	}

	void RigidTerrain::readChannelData()
	{

		// Clear normal and posiiton data
		mNormals.clear();
		mPositions.clear();

		// Get image dimensions
		int32_t height		= mChannel.getHeight();
		int32_t width		= mChannel.getWidth();
		float halfHeight	= (float)height * 0.5f;
		float halfWidth =	(float)width * 0.5f;
		
		// Iterate through dimensions
		for ( int32_t y = 0; y < height; y++ ) {
			for ( int32_t x = 0; x < width; x++ ) {

				// Get pixel shade
				float value = mChannel.getValue( Vec2i( x, y ) );
				
				// Add position
				Vec3f position( (float)x - halfWidth, value, (float)y - halfHeight );
				mPositions.push_back( position );

				// Add default normal
				mNormals.push_back( Vec3f::zero() );

			}

		}

		// Loop through again to set normals
		for ( int32_t y = 0; y < height - 1; y++ ) {
			for (int32_t x = 0; x < width - 1; x++) {

				// Get vertices of this triangle
				Vec3f vert0 = mPositions[ mIndices[ ( x + height * y ) * 6 ] ];
				Vec3f vert1 = mPositions[ mIndices[ ( ( x + 1 ) + height * y ) * 6 ] ];
				Vec3f vert2 = mPositions[ mIndices[ ( ( x + 1 ) + height * ( y + 1 ) ) * 6 ] ];

				// Calculate normal
				mNormals[ x + height * y ] = Vec3f( ( vert1 - vert0 ).cross( vert1 - vert2 ).normalized() );

			}
		}

	}

}
