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
#include "RigidObject.h"

#include "cinder/Utilities.h"

namespace bullet
{

	// Import namespaces
	using namespace ci;
	using namespace std;

	// Creates rigid body from any collision shape
	btRigidBody* RigidObject::create( btCollisionShape* shape, float mass, const Vec3f &position, const Quatf &rotation )
	{

		// Calculate inertia
		btVector3 inertia( 0.0f, 0.0f, 0.0f );
		if ( mass != 0.0f ) {
			shape->calculateLocalInertia( mass, inertia );
		}

		// Define bullet body
		btQuaternion btRotation = Utilities::toBulletQuaternion( rotation );
		btVector3 btPosition = Utilities::toBulletVector3( position );
		btTransform worldTransform( btRotation, btPosition );
		btDefaultMotionState* motionState = new btDefaultMotionState( worldTransform );
		btRigidBody::btRigidBodyConstructionInfo info( mass, motionState, shape, inertia );

		// Create and return body
		btRigidBody* body = new btRigidBody( info );
		return body;

	}

	// Creates a rigid box
	btRigidBody* RigidObject::createBox( const Vec3f& size, float mass, const Vec3f& position, const Quatf& rotation )
	{

		// Create Bullet box
		btVector3 halfSize = Utilities::toBulletVector3( size ) * 0.5f;
		btCollisionShape* shape = new btBoxShape( halfSize );

		// Create and return rigid body
		btRigidBody* body = create( shape, mass, position, rotation );
		return body;

	}

	// Creates rigid cylinder
	btRigidBody* RigidObject::createCylinder( float topRadius, float bottomRadius, float height, int32_t segments, float mass, const Vec3f& position, const Quatf& rotation )
	{

		// Create cylinder
		btVector3 size = btVector3( height, topRadius, bottomRadius );
		btCollisionShape* shape = new btCylinderShape( size );

		// Create and return rigid body
		btRigidBody* body = create( shape, mass, position, rotation );
		return body;

	}

	// Create rigid body from convex hull shape
	btRigidBody* RigidObject::createHull( btConvexHullShape* shape, float mass, const Vec3f& position, const Quatf& rotation )
	{

		// Create and return rigid body
		btRigidBody* body = create( shape, mass, position, rotation );
		return body;

	}

	/*! Create rigid body from triangle mesh */
	btRigidBody* RigidObject::createMesh( btBvhTriangleMeshShape* shape, float mass, const Vec3f& position, const Quatf& rotation )
	{

		// Create and return rigid body
		btRigidBody* body = create( shape, mass, position, rotation );
		return body;

	}

	// Creates a rigid sphere
	btRigidBody* RigidObject::createSphere( float radius, float mass, const Vec3f& position, const Quatf& rotation )
	{

		// Create Bullet sphere
		btCollisionShape* shape = new btSphereShape( ( btScalar ) radius );

		// Create and return rigid body
		btRigidBody* body = create( shape, mass, position, rotation );
		return body;

	}

	// Creates terrain
	btRigidBody* RigidObject::createTerrain( btHeightfieldTerrainShape* shape, float mass, const ci::Vec3f &position, const ci::Quatf &rotation )
	{

		// Create and return rigid body
		btRigidBody* body = create( shape, mass, position, rotation );
		return body;

	}

	RigidBox::RigidBox( const Vec3f &dimensions, float mass, const Vec3f &position, const Quatf &rotation ) 
		: CollisionObjectBase( position, rotation )
	{

		// Create body
		mRigidBody = RigidObject::createBox( dimensions, mass, position, rotation );

		Vec3f size = dimensions * 0.5f;
		
		Vec3f pos0 = Vec3f( 1.0f, 1.0f, 1.0f ) * size;
		Vec3f pos1 = Vec3f( 1.0f, -1.0f, 1.0f ) * size;
		Vec3f pos2 = Vec3f( 1.0f, -1.0f, -1.0f ) * size;
		Vec3f pos3 = Vec3f( 1.0f, 1.0f, -1.0f ) * size;
		Vec3f pos4 = Vec3f( -1.0f, 1.0f, -1.0f ) * size;
		Vec3f pos5 = Vec3f( -1.0f, 1.0f, 1.0f ) * size;
		Vec3f pos6 = Vec3f( -1.0f, -1.0f, -1.0f ) * size;
		Vec3f pos7 = Vec3f( -1.0f, -1.0f, 1.0f ) * size;

		mVboPositions.push_back( pos0 ); 
		mVboPositions.push_back( pos1 ); 	
		mVboPositions.push_back( pos2 ); 	
		mVboPositions.push_back( pos3 ); // +X

		mVboPositions.push_back( pos0 ); 
		mVboPositions.push_back( pos3 ); 	
		mVboPositions.push_back( pos4 ); 	
		mVboPositions.push_back( pos5 ); // +Y

		mVboPositions.push_back( pos0 ); 	
		mVboPositions.push_back( pos5 ); 	
		mVboPositions.push_back( pos7 ); 	
		mVboPositions.push_back( pos1 ); // +Z

		mVboPositions.push_back( pos5 ); 	
		mVboPositions.push_back( pos4 ); 	
		mVboPositions.push_back( pos6 ); 	
		mVboPositions.push_back( Vec3f( -1.0f, -1.0f, 1.0f ) * size ); // -X

		mVboPositions.push_back( pos6 ); 	
		mVboPositions.push_back( pos2 ); 	
		mVboPositions.push_back( pos1 ); 	
		mVboPositions.push_back( Vec3f( -1.0f, -1.0f, 1.0f ) * size ); // -Y

		mVboPositions.push_back( pos2 ); 	
		mVboPositions.push_back( pos6 ); 	
		mVboPositions.push_back( pos4 ); 	
		mVboPositions.push_back( pos3 ); // -Z

		Vec3f norm0( 1.0f, 0.0f, 0.0f );
		Vec3f norm1( 0.0f, 1.0f, 0.0f );
		Vec3f norm2( 0.0f, 0.0f, 1.0f );
		Vec3f norm3( -1.0f, 0.0f, 0.0f ); 
		Vec3f norm4( 0.0f, -1.0f, 0.0f ); 
		Vec3f norm5( 0.0f, 0.0f, -1.0f ); 
		for ( uint8_t i = 0; i < 4; i++ ) {
			mVboNormals.push_back( norm0 );
		}
		for ( uint8_t i = 0; i < 4; i++ ) {
			mVboNormals.push_back( norm1 );
		}
		for ( uint8_t i = 0; i < 4; i++ ) {
			mVboNormals.push_back( norm2 );
		}
		for ( uint8_t i = 0; i < 4; i++ ) {
			mVboNormals.push_back( norm3 );
		}
		for ( uint8_t i = 0; i < 4; i++ ) {
			mVboNormals.push_back( norm4 );
		}
		for ( uint8_t i = 0; i < 4; i++ ) {
			mVboNormals.push_back( norm5 );
		}

		

		/*static GLfloat texs[24*2]={	0,1,	1,1,	1,0,	0,0,
			1,1,	1,0,	0,0,	0,1,
			0,1,	1,1,	1,0,	0,0,							
			1,1,	1,0,	0,0,	0,1,
			1,0,	0,0,	0,1,	1,1,
			1,0,	0,0,	0,1,	1,1 };*/

		mVboIndices.push_back( 0 );
		mVboIndices.push_back( 1 );
		mVboIndices.push_back( 2 );
		mVboIndices.push_back( 0 );
		mVboIndices.push_back( 2 );
		mVboIndices.push_back( 3 );
		
		mVboIndices.push_back( 4 );
		mVboIndices.push_back( 5 );
		mVboIndices.push_back( 6 );
		mVboIndices.push_back( 4 );
		mVboIndices.push_back( 6 );
		mVboIndices.push_back( 7 );
		
		mVboIndices.push_back( 8 );
		mVboIndices.push_back( 9 );
		mVboIndices.push_back( 10 );
		mVboIndices.push_back( 8 );
		mVboIndices.push_back( 10 );
		mVboIndices.push_back( 11 );
		
		mVboIndices.push_back( 12 );
		mVboIndices.push_back( 13 );
		mVboIndices.push_back( 14 );
		mVboIndices.push_back( 12 );
		mVboIndices.push_back( 14 );
		mVboIndices.push_back( 15 );
		
		mVboIndices.push_back( 16 );
		mVboIndices.push_back( 17 );
		mVboIndices.push_back( 18 );
		mVboIndices.push_back( 16 );
		mVboIndices.push_back( 18 );
		mVboIndices.push_back( 19 );
		
		mVboIndices.push_back( 20 );
		mVboIndices.push_back( 21 );
		mVboIndices.push_back( 22 );
		mVboIndices.push_back( 20 );
		mVboIndices.push_back( 22 );
		mVboIndices.push_back( 23 );

		/*

		// Define vertices
		Vec3f size = dimensions * 0.5f;
		mVboPositions.push_back( size * Vec3f( -1.0f, -1.0f, -1.0f ) ); // 0 ---
		mVboPositions.push_back( size * Vec3f(  1.0f, -1.0f, -1.0f ) ); // 1 +--
		mVboPositions.push_back( size * Vec3f( -1.0f,  1.0f, -1.0f ) ); // 2 -+-
		mVboPositions.push_back( size * Vec3f(  1.0f,  1.0f, -1.0f ) ); // 3 ++-
		mVboPositions.push_back( size * Vec3f( -1.0f, -1.0f,  1.0f ) ); // 4 --+
		mVboPositions.push_back( size * Vec3f(  1.0f, -1.0f,  1.0f ) ); // 5 +-+
		mVboPositions.push_back( size * Vec3f( -1.0f,  1.0f,  1.0f ) ); // 6 -++
		mVboPositions.push_back( size * Vec3f(  1.0f,  1.0f,  1.0f ) ); // 7 +++

		// Define normals
		Vec3f norm0 = Vec3f(  1.0f,  0.0f,  0.0f ); // Right
		Vec3f norm1 = Vec3f(  0.0f,  1.0f,  0.0f ); // Top
		Vec3f norm2 = Vec3f(  0.0f,  0.0f,  1.0f ); // Front
		Vec3f norm3 = Vec3f( -1.0f,  0.0f,  0.0f ); // Left
		Vec3f norm4 = Vec3f(  0.0f, -1.0f,  0.0f ); // Bottom
		Vec3f norm5 = Vec3f(  0.0f,  0.0f, -1.0f ); // Back

		// Set normals
		for ( int32_t i = 0; i < 4; i++ ) {
			mVboNormals.push_back( norm0 );
		}
		for ( int32_t i = 0; i < 4; i++ ) {
			mVboNormals.push_back( norm1 );
		}
		for ( int32_t i = 0; i < 4; i++ ) {
			mVboNormals.push_back( norm2 );
		}
		for ( int32_t i = 0; i < 4; i++ ) {
			mVboNormals.push_back( norm3 );
		}
		for ( int32_t i = 0; i < 4; i++ ) {
			mVboNormals.push_back( norm4 );
		}
		for ( int32_t i = 0; i < 4; i++ ) {
			mVboNormals.push_back( norm5 );
		}

		// Right
		mVboIndices.push_back( 1 );
		mVboIndices.push_back( 3 );
		mVboIndices.push_back( 5 );
		mVboIndices.push_back( 5 );
		mVboIndices.push_back( 3 );
		mVboIndices.push_back( 7 );

		// Top
		mVboIndices.push_back( 2 );
		mVboIndices.push_back( 3 );
		mVboIndices.push_back( 6 );
		mVboIndices.push_back( 6 );
		mVboIndices.push_back( 3 );
		mVboIndices.push_back( 7 );

		// Front
		mVboIndices.push_back( 4 );
		mVboIndices.push_back( 5 );
		mVboIndices.push_back( 6 );
		mVboIndices.push_back( 6 );
		mVboIndices.push_back( 5 );
		mVboIndices.push_back( 7 );

		// Left
		mVboIndices.push_back( 0 );
		mVboIndices.push_back( 2 );
		mVboIndices.push_back( 4 );
		mVboIndices.push_back( 4 );
		mVboIndices.push_back( 2 );
		mVboIndices.push_back( 6 );

		// Bottom
		mVboIndices.push_back( 0 );
		mVboIndices.push_back( 1 );
		mVboIndices.push_back( 4 );
		mVboIndices.push_back( 4 );
		mVboIndices.push_back( 1 );
		mVboIndices.push_back( 5 );

		// Back
		mVboIndices.push_back( 0 );
		mVboIndices.push_back( 1 );
		mVboIndices.push_back( 2 );
		mVboIndices.push_back( 2 );
		mVboIndices.push_back( 1 );
		mVboIndices.push_back( 3 );*/

		// Set VBO data
		setVboData();
		clearVboData();

	}

	RigidCylinder::RigidCylinder( float topRadius, float bottomRadius, float height, int32_t segments, 
		float mass, const Vec3f &position, const Quatf &rotation )
		: CollisionObjectBase( position, rotation )
	{

		// Create body
		mRigidBody = RigidObject::createCylinder( topRadius, bottomRadius, height, segments, mass, position, rotation );

		// Set delta size
		float delta = 1.0f / (float)segments;

		// Iterate layers
		for ( int32_t p = 0; p < 2; p++ ) {

			// Choose radius
			float radius = p == 0 ? bottomRadius : topRadius;

			// Iterate segments
			int32_t t = 0;
			for ( float theta = delta; t < segments; t++, theta += delta ) {

				// Set normal
				Vec3f normal( math<float>::cos( 2.0f * (float)M_PI * theta ), 0.0f, math<float>::sin( 2.0f * (float)M_PI * theta ) );
				mVboNormals.push_back( normal );

				// Set vertex
				Vec3f position( normal.x * radius, (float)p * height, normal.z * radius );
				mVboPositions.push_back( position );

			}

		}

		// Top and bottom center
		mVboPositions.push_back( Vec3f::zero() );
		mVboPositions.push_back( Vec3f( 0.0f, height, 0.0f ) );
		int32_t bottomCenter = (int32_t)mVboPositions.size() - 1;
		int32_t topCenter = bottomCenter - 1;

		// Build top face
		for ( int32_t t = 0; t < segments; t++ ) {
			int32_t n = t + 1 >= segments ? 0 : t + 1;
			mVboIndices.push_back( t );
			mVboIndices.push_back( topCenter );
			mVboIndices.push_back( n );
		}

		// Build body
		for ( int32_t t = 0; t < segments; t++ ) {
			int32_t n = t + 1 >= segments ? 0 : t + 1;
			mVboIndices.push_back( t );
			mVboIndices.push_back( segments + t );
			mVboIndices.push_back( n );
			mVboIndices.push_back( n );
			mVboIndices.push_back( segments + t );
			mVboIndices.push_back( segments + n );
		}
			
		// Build bottom face
		for ( int32_t t = 0; t < segments; t++ ) {
			int32_t n = t + 1 >= segments ? 0 : t + 1;
			mVboIndices.push_back( segments + t );
			mVboIndices.push_back( bottomCenter );
			mVboIndices.push_back( segments + n );
		}
		
		// Set VBO data
		setVboData();
		clearVboData();

	}

	// Convex hull mesh
	RigidHull::RigidHull( const TriMesh &mesh, const Vec3f &scale, float mass, const Vec3f &position, const Quatf &rotation )
		: CollisionObjectBase( position, rotation )
	{

		// Change positions to dynamic
		mVboLayout.setDynamicPositions();

		// Define body and VBO
		mRigidBody = RigidObject::createHull( Utilities::createConvexHullShape( mesh, scale ), mass, position, rotation );
		mVboMesh = gl::VboMesh( mesh, mVboLayout );

		// Scale VBO
		gl::VboMesh::VertexIter vertexIt = mVboMesh.mapVertexBuffer();
		for ( uint32_t i = 0; i < mVboMesh.getNumVertices(); i++, ++vertexIt ) {
			vertexIt.setPosition( vertexIt.getPositionPointer()->xyz() * scale );
		}

	}

	// Concave mesh
	RigidMesh::RigidMesh( const TriMesh &mesh, const Vec3f &scale, float margin, float mass, const Vec3f &position, const Quatf &rotation )
		: CollisionObjectBase( position, rotation )
	{

		// Change positions to dynamic so we can scale the VBO
		mVboLayout.setDynamicPositions();

		// Define body and VBO
		Vec3f halfScale = scale * 0.5f;
		btBvhTriangleMeshShape* shape = Utilities::createConcaveMeshShape( mesh, scale, margin );
		mRigidBody = RigidObject::createMesh( shape, mass, position, rotation );
		mVboMesh = gl::VboMesh( mesh, mVboLayout );

		// Scale VBO
		gl::VboMesh::VertexIter vertexIt = mVboMesh.mapVertexBuffer();
		for ( uint32_t i = 0; i < mVboMesh.getNumVertices(); i++, ++vertexIt ) {
			vertexIt.setPosition( vertexIt.getPositionPointer()->xyz() * scale );
		}

	}

	// Sphere
	RigidSphere::RigidSphere( float radius, int32_t segments, float mass, const Vec3f &position, const Quatf &rotation ) 
		: CollisionObjectBase( position, rotation )
	{

		// Create body
		mRigidBody = RigidObject::createSphere( radius, mass, position, rotation );

		// Define steps
		int32_t layers = segments / 2;
		float step = (float)M_PI / (float)layers;
		float delta = ((float)M_PI * 2.0f) / (float)segments;

		// Phi
		int32_t p = 0;
		for ( float phi = 0.0f; p <= layers; p++, phi += step ) {

			// Theta
			int32_t t = 0;
			for ( float theta = delta; t < segments; t++, theta += delta )
			{

				// Set position
				Vec3f position(
					radius * math<float>::sin( phi ) * math<float>::cos( theta ),
					radius * math<float>::sin( phi ) * math<float>::sin( theta ),
					-radius * math<float>::cos( phi ) );
				mVboPositions.push_back(position);

				// Set normal
				Vec3f normal = position.normalized();
				mVboNormals.push_back( normal );

				// Add indices
				int32_t n = t + 1 >= segments ? 0 : t + 1;
				mVboIndices.push_back( p * segments + t );
				mVboIndices.push_back( ( p + 1 ) * segments + t );
				mVboIndices.push_back( p * segments + n );
				mVboIndices.push_back( p * segments + n );
				mVboIndices.push_back( ( p + 1 ) * segments + t );
				mVboIndices.push_back( ( p + 1 ) * segments + n );

			}

		}

		// Set VBO data
		setVboData();
		clearVboData();

	}

	// Terrain from Surface data
	RigidTerrain::RigidTerrain( const Surface32f &heightField, int32_t stickWidth, int32_t stickLength, float minHeight, float maxHeight, 
		int32_t upAxis, const Vec3f &scale, float mass, const Vec3f &position, const Quatf &rotation )
		: CollisionObjectBase( position, rotation )
	{

		// Create body
		btHeightfieldTerrainShape* shape = Utilities::createHeightfieldTerrainShape( heightField, stickWidth, stickLength, 1.0f, minHeight, maxHeight, 1, scale );
		mRigidBody = RigidObject::createTerrain( shape, mass, position, rotation );

		// Get image dimensions
		int32_t height = heightField.getHeight();
		int32_t width = heightField.getWidth();
		float halfHeight = (float)height * 0.5f;
		float halfWidth = (float)width * 0.5f;
		
		// Set grey for dot product
		Vec3f grey( 0.3333f, 0.3333f, 0.3333f );

		// Iterate through dimensions
		for ( int32_t y = 0; y < height; y++ ) {
			for ( int32_t x = 0; x < width; x++ ) {

				// Get pixel color
				Colorf color( heightField.getPixel( Vec2i( x, y ) ) );
				
				// Add position
				mVboPositions.push_back( Vec3f(
					( (float)x - halfWidth ) * scale.x, 
					Vec3f( color.r, color.g, color.b ).dot(grey) * scale.y, 
					( ( float)y - halfHeight ) * scale.z )
					);

				// Add default normal
				mVboNormals.push_back( Vec3f::zero() );

				// Add indices for this quad
				int32_t xn = x + 1 >= width ? 0 : 1;
				int32_t yn = y + 1 >= height ? 0 : 1;
				mVboIndices.push_back( x + height * y );
				mVboIndices.push_back( ( x + xn ) + height * y);
				mVboIndices.push_back( ( x + xn ) + height * ( y + yn ) );
				mVboIndices.push_back( x + height * ( y + yn ) );
				mVboIndices.push_back( ( x + xn ) + height * ( y + yn ) );
				mVboIndices.push_back( x + height * y );

			}

		}

		// Loop through again to set normals
		for ( int32_t y = 0; y < height - 1; y++ ) {
			for (int32_t x = 0; x < width - 1; x++) {

				// Get vertices of this triangle
				Vec3f vert0 = mVboPositions[ mVboIndices[ ( x + height * y ) * 6 ] ];
				Vec3f vert1 = mVboPositions[ mVboIndices[ ( ( x + 1 ) + height * y ) * 6 ] ];
				Vec3f vert2 = mVboPositions[ mVboIndices[ ( ( x + 1 ) + height * ( y + 1 ) ) * 6 ] ];

				// Calculate normal
				mVboNormals[ x + height * y ] = Vec3f( ( vert1 - vert0 ).cross( vert1 - vert2 ).normalized() );

			}
		}

		// Set VBO data
		setVboData();
		clearVboData();

	}

	// Torus
	RigidTorus::RigidTorus( float innerRadius, float outerRadius, int32_t segments, float mass, const Vec3f &position, const Quatf &rotation ) 
		: CollisionObjectBase( position, rotation)
	{

		// TO DO create torus TriMesh
		TriMesh mesh;

		// Define bullet body
		mRigidBody = RigidObject::createMesh( Utilities::createConcaveMeshShape( mesh, Vec3f::one(), 0.0f ), mass, position, rotation );

		// Set VBO
		mVboMesh = gl::VboMesh( mesh, mVboLayout );

	}

}
