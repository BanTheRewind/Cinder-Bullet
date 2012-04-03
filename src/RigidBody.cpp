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
		btQuaternion btRotation = toBulletQuaternion( rotation );
		btVector3 btPosition = toBulletVector3( position );
		btTransform worldTransform( btRotation, btPosition );
		btDefaultMotionState* motionState = new btDefaultMotionState( worldTransform );
		btRigidBody::btRigidBodyConstructionInfo info( mass, motionState, shape, inertia );

		// Create and return body
		btRigidBody* body = new btRigidBody( info );
		return body;

	}

	// Creates a rigid box
	btRigidBody* RigidBody::createBox( const Vec3f& size, float mass, const Vec3f& position, const Quatf& rotation )
	{

		// Create Bullet box
		btVector3 halfSize = toBulletVector3( size ) * 0.5f;
		btCollisionShape* shape = new btBoxShape( halfSize );

		// Create and return rigid body
		btRigidBody* body = create( shape, mass, position, rotation );
		return body;

	}

	// Creates rigid cylinder
	btRigidBody* RigidBody::createCylinder( float topRadius, float bottomRadius, float height, int32_t segments, float mass, const Vec3f& position, const Quatf& rotation )
	{

		// Create cylinder
		btVector3 size = btVector3( height, topRadius, bottomRadius );
		btCollisionShape* shape = new btCylinderShape( size );

		// Create and return rigid body
		btRigidBody* body = create( shape, mass, position, rotation );
		return body;

	}

	// Create rigid body from convex hull shape
	btRigidBody* RigidBody::createHull( btConvexHullShape* shape, float mass, const Vec3f& position, const Quatf& rotation )
	{

		// Create and return rigid body
		btRigidBody* body = create( shape, mass, position, rotation );
		return body;

	}

	/*! Create rigid body from triangle mesh */
	btRigidBody* RigidBody::createMesh( btBvhTriangleMeshShape* shape, float mass, const Vec3f& position, const Quatf& rotation )
	{

		// Create and return rigid body
		btRigidBody* body = create( shape, mass, position, rotation );
		return body;

	}

	// Creates a rigid sphere
	btRigidBody* RigidBody::createSphere( float radius, float mass, const Vec3f& position, const Quatf& rotation )
	{

		// Create Bullet sphere
		btCollisionShape* shape = new btSphereShape( ( btScalar ) radius );

		// Create and return rigid body
		btRigidBody* body = create( shape, mass, position, rotation );
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
		mRigidBody = RigidBody::createBox( dimensions, mass, position, rotation );

		// Declare vectors
		vector<uint32_t> indices;
		vector<Vec3f> normals;
		vector<Vec3f> positions;
		vector<Vec2f> texCoords;

		// Define VBO positions
		Vec3f size = dimensions * 0.5f;
		Vec3f pos0 = Vec3f( 1.0f, 1.0f, 1.0f ) * size;
		Vec3f pos1 = Vec3f( 1.0f, -1.0f, 1.0f ) * size;
		Vec3f pos2 = Vec3f( 1.0f, -1.0f, -1.0f ) * size;
		Vec3f pos3 = Vec3f( 1.0f, 1.0f, -1.0f ) * size;
		Vec3f pos4 = Vec3f( -1.0f, 1.0f, -1.0f ) * size;
		Vec3f pos5 = Vec3f( -1.0f, 1.0f, 1.0f ) * size;
		Vec3f pos6 = Vec3f( -1.0f, -1.0f, -1.0f ) * size;
		Vec3f pos7 = Vec3f( -1.0f, -1.0f, 1.0f ) * size;

		// Define nromals
		Vec3f norm0( 1.0f, 0.0f, 0.0f );
		Vec3f norm1( 0.0f, 1.0f, 0.0f );
		Vec3f norm2( 0.0f, 0.0f, 1.0f );
		Vec3f norm3( -1.0f, 0.0f, 0.0f ); 
		Vec3f norm4( 0.0f, -1.0f, 0.0f ); 
		Vec3f norm5( 0.0f, 0.0f, -1.0f ); 

		// Add positions
		positions.push_back( pos0 ); 
		positions.push_back( pos1 ); 	
		positions.push_back( pos2 ); 	
		positions.push_back( pos3 );

		positions.push_back( pos0 ); 
		positions.push_back( pos3 ); 	
		positions.push_back( pos4 ); 	
		positions.push_back( pos5 );

		positions.push_back( pos0 ); 	
		positions.push_back( pos5 ); 	
		positions.push_back( pos7 ); 	
		positions.push_back( pos1 );

		positions.push_back( pos5 ); 	
		positions.push_back( pos4 ); 	
		positions.push_back( pos6 ); 	
		positions.push_back( pos7 );

		positions.push_back( pos6 ); 	
		positions.push_back( pos2 ); 	
		positions.push_back( pos1 ); 	
		positions.push_back( pos7 );

		positions.push_back( pos2 ); 	
		positions.push_back( pos6 ); 	
		positions.push_back( pos4 ); 	
		positions.push_back( pos3 );

		// Add normals
		for ( uint8_t i = 0; i < 4; i++ ) {
			normals.push_back( norm0 );
		}
		for ( uint8_t i = 0; i < 4; i++ ) {
			normals.push_back( norm1 );
		}
		for ( uint8_t i = 0; i < 4; i++ ) {
			normals.push_back( norm2 );
		}
		for ( uint8_t i = 0; i < 4; i++ ) {
			normals.push_back( norm3 );
		}
		for ( uint8_t i = 0; i < 4; i++ ) {
			normals.push_back( norm4 );
		}
		for ( uint8_t i = 0; i < 4; i++ ) {
			normals.push_back( norm5 );
		}

		// Define texture coordinates
		Vec2f texCoord0( 0.0f, 0.0f );
		Vec2f texCoord1( 1.0f, 0.0f );
		Vec2f texCoord2( 1.0f, 1.0f );
		Vec2f texCoord3( 0.0f, 1.0f );

		// Add texture coordinates
		texCoords.push_back( texCoord3 );
		texCoords.push_back( texCoord2 );
		texCoords.push_back( texCoord1 );
		texCoords.push_back( texCoord0 );

		texCoords.push_back( texCoord2 );
		texCoords.push_back( texCoord1 );
		texCoords.push_back( texCoord0 );
		texCoords.push_back( texCoord3 );

		texCoords.push_back( texCoord3 );
		texCoords.push_back( texCoord2 );
		texCoords.push_back( texCoord1 );
		texCoords.push_back( texCoord0 );

		texCoords.push_back( texCoord2 );
		texCoords.push_back( texCoord1 );
		texCoords.push_back( texCoord0 );
		texCoords.push_back( texCoord3 );

		texCoords.push_back( texCoord1 );
		texCoords.push_back( texCoord0 );
		texCoords.push_back( texCoord3 );
		texCoords.push_back( texCoord2 );
		
		texCoords.push_back( texCoord1 );
		texCoords.push_back( texCoord0 );			
		texCoords.push_back( texCoord3 );
		texCoords.push_back( texCoord2 );

		// Add indices
		indices.push_back( 0 );
		indices.push_back( 1 );
		indices.push_back( 2 );
		indices.push_back( 0 );
		indices.push_back( 2 );
		indices.push_back( 3 );
		
		indices.push_back( 4 );
		indices.push_back( 5 );
		indices.push_back( 6 );
		indices.push_back( 4 );
		indices.push_back( 6 );
		indices.push_back( 7 );
		
		indices.push_back( 8 );
		indices.push_back( 9 );
		indices.push_back( 10 );
		indices.push_back( 8 );
		indices.push_back( 10 );
		indices.push_back( 11 );
		
		indices.push_back( 12 );
		indices.push_back( 13 );
		indices.push_back( 14 );
		indices.push_back( 12 );
		indices.push_back( 14 );
		indices.push_back( 15 );
		
		indices.push_back( 16 );
		indices.push_back( 17 );
		indices.push_back( 18 );
		indices.push_back( 16 );
		indices.push_back( 18 );
		indices.push_back( 19 );
		
		indices.push_back( 20 );
		indices.push_back( 21 );
		indices.push_back( 22 );
		indices.push_back( 20 );
		indices.push_back( 22 );
		indices.push_back( 23 );

		// Set VBO data
		setVboData( indices, positions, normals, texCoords );

		// Clean up
		indices.clear();
		normals.clear();
		positions.clear();
		texCoords.clear();

	}

	RigidCylinder::RigidCylinder( float topRadius, float bottomRadius, float height, int32_t segments, 
		float mass, const Vec3f &position, const Quatf &rotation )
		: CollisionObject( position, rotation )
	{

		// Create body
		mRigidBody = RigidBody::createCylinder( topRadius, bottomRadius, height, segments, mass, position, rotation );

		// Declare vectors
		vector<uint32_t> indices;
		vector<Vec3f> normals;
		vector<Vec3f> positions;
		vector<Vec2f> texCoords;

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
				Vec3f normal( math<float>::cos( theta ), 0.0f, math<float>::sin( theta ) );
				normals.push_back( normal );

				// Set vertex
				float t = 2.0f * (float)M_PI * theta;
				Vec3f position( 
					math<float>::cos( t ) * radius, 
					(float)p * height, 
					math<float>::sin( t ) * radius 
					);
				positions.push_back( position );

			}

		}

		// Top and bottom center
		positions.push_back( Vec3f::zero() );
		positions.push_back( Vec3f( 0.0f, height, 0.0f ) );
		int32_t bottomCenter = (int32_t)positions.size() - 1;
		int32_t topCenter = bottomCenter - 1;

		// Build top face
		for ( int32_t t = 0; t < segments; t++ ) {
			int32_t n = t + 1 >= segments ? 0 : t + 1;
			indices.push_back( t );
			indices.push_back( topCenter );
			indices.push_back( n );
		}

		// Build body
		for ( int32_t t = 0; t < segments; t++ ) {
			int32_t n = t + 1 >= segments ? 0 : t + 1;
			indices.push_back( t );
			indices.push_back( segments + t );
			indices.push_back( n );
			indices.push_back( n );
			indices.push_back( segments + t );
			indices.push_back( segments + n );
		}
			
		// Build bottom face
		for ( int32_t t = 0; t < segments; t++ ) {
			int32_t n = t + 1 >= segments ? 0 : t + 1;
			indices.push_back( segments + t );
			indices.push_back( bottomCenter );
			indices.push_back( segments + n );
		}
		
		// Set VBO data
		setVboData( indices, positions, normals, texCoords, GL_TRIANGLE_STRIP );

		// Clean up
		indices.clear();
		normals.clear();
		positions.clear();
		texCoords.clear();

	}

	// Convex hull mesh
	RigidHull::RigidHull( const TriMesh &mesh, const Vec3f &scale, float mass, const Vec3f &position, const Quatf &rotation )
		: CollisionObject( position, rotation )
	{

		// Scale mesh to size of body
		vector<Vec3f> positions;
		for ( vector<Vec3f>::const_iterator posIt = mesh.getVertices().begin(); posIt != mesh.getVertices().end(); ++posIt ) {
			positions.push_back( *posIt * scale );
		}

		// Define body
		btConvexHullShape* shape = createConvexHullShape( mesh.getVertices(), scale );
		mRigidBody = RigidBody::createHull( shape, mass, position, rotation );

		// Set VBO data
		setVboData( mesh.getIndices(), positions, mesh.getNormals(), mesh.getTexCoords() );

		// Clean up
		positions.clear();

	}

	// Concave mesh
	RigidMesh::RigidMesh( const TriMesh &mesh, const Vec3f &scale, float margin, float mass, const Vec3f &position, const Quatf &rotation )
		: CollisionObject( position, rotation )
	{

		// Declare vectors
		vector<Vec3f> positions;
		
		// Define body
		Vec3f halfScale = scale * 0.5f;
		btBvhTriangleMeshShape* shape = createConcaveMeshShape( mesh.getVertices(), mesh.getIndices(), scale, margin );
		mRigidBody = RigidBody::createMesh( shape, mass, position, rotation );
		
		// Scale mesh to size of body
		for ( vector<Vec3f>::const_iterator posIt = mesh.getVertices().begin(); posIt != mesh.getVertices().end(); ++posIt ) {
			positions.push_back( *posIt * scale );
		}

		// Set VBO data
		setVboData( mesh.getIndices(), positions, mesh.getNormals(), mesh.getTexCoords() );

		// Clean up
		positions.clear();
		
	}

	// Sphere
	RigidSphere::RigidSphere( float radius, int32_t segments, float mass, const Vec3f &position, const Quatf &rotation ) 
		: CollisionObject( position, rotation )
	{

		// Create body
		mRigidBody = RigidBody::createSphere( radius, mass, position, rotation );

		// Declare vectors
		vector<uint32_t> indices;
		vector<Vec3f> normals;
		vector<Vec3f> positions;
		vector<Vec2f> texCoords;

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
				positions.push_back(position);

				// Set normal
				Vec3f normal = position.normalized();
				normals.push_back( normal );

				// Add indices
				int32_t n = t + 1 >= segments ? 0 : t + 1;
				indices.push_back( p * segments + t );
				indices.push_back( ( p + 1 ) * segments + t );
				indices.push_back( p * segments + n );
				indices.push_back( p * segments + n );
				indices.push_back( ( p + 1 ) * segments + t );
				indices.push_back( ( p + 1 ) * segments + n );

			}

		}

		// Set VBO data
		setVboData( indices, positions, normals, texCoords );

		// Clean up
		indices.clear();
		normals.clear();
		positions.clear();
		texCoords.clear();

	}

	// Terrain from Surface data
	RigidTerrain::RigidTerrain( const Surface32f &heightField, int32_t stickWidth, int32_t stickLength, float minHeight, float maxHeight, 
		int32_t upAxis, const Vec3f &scale, float mass, const Vec3f &position, const Quatf &rotation )
		: CollisionObject( position, rotation )
	{

		// Create body
		btHeightfieldTerrainShape* shape = createHeightfieldTerrainShape( heightField, stickWidth, stickLength, 1.0f, minHeight, maxHeight, 1, scale );
		mRigidBody = RigidBody::createTerrain( shape, mass, position, rotation );

		// Declare vectors
		vector<uint32_t> indices;
		vector<Vec3f> normals;
		vector<Vec3f> positions;
		vector<Vec2f> texCoords;

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
				positions.push_back( Vec3f(
					( (float)x - halfWidth ) * scale.x, 
					Vec3f( color.r, color.g, color.b ).dot(grey) * scale.y, 
					( ( float)y - halfHeight ) * scale.z )
					);

				// Add default normal
				normals.push_back( Vec3f::zero() );

				// Add indices for this quad
				int32_t xn = x + 1 >= width ? 0 : 1;
				int32_t yn = y + 1 >= height ? 0 : 1;
				indices.push_back( x + height * y );
				indices.push_back( ( x + xn ) + height * y);
				indices.push_back( ( x + xn ) + height * ( y + yn ) );
				indices.push_back( x + height * ( y + yn ) );
				indices.push_back( ( x + xn ) + height * ( y + yn ) );
				indices.push_back( x + height * y );

			}

		}

		// Loop through again to set normals
		for ( int32_t y = 0; y < height - 1; y++ ) {
			for (int32_t x = 0; x < width - 1; x++) {

				// Get vertices of this triangle
				Vec3f vert0 = positions[ indices[ ( x + height * y ) * 6 ] ];
				Vec3f vert1 = positions[ indices[ ( ( x + 1 ) + height * y ) * 6 ] ];
				Vec3f vert2 = positions[ indices[ ( ( x + 1 ) + height * ( y + 1 ) ) * 6 ] ];

				// Calculate normal
				normals[ x + height * y ] = Vec3f( ( vert1 - vert0 ).cross( vert1 - vert2 ).normalized() );

			}
		}

		// Set VBO data
		setVboData( indices, positions, normals, texCoords );

		// Clean up
		indices.clear();
		normals.clear();
		positions.clear();
		texCoords.clear();

	}
}
