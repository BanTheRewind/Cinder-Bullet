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
#include "Utilities.h"

namespace bullet 
{

	// Imports
	using namespace ci;
	using namespace std;

	/////////////////////////////////////////////////////////////////////////////////////////////////

	// Convert Bullet quaternion to Cinder quaternion
	Quatf fromBulletQuaternion( const btQuaternion& q )
	{
		return ci::Quatf( q.getX(), q.getY(), q.getZ(), q.getW() );
	}

	Matrix44f fromBulletTransform( const btTransform &m )
	{
		btTransform trans;
		Matrix44f matrix;
		m.getOpenGLMatrix( matrix.m );
		return matrix;
	}

	// Convert Bullet vector to Cinder vector
	Vec3f fromBulletVector3( const btVector3& v )
	{
		return Vec3f( v.x(), v.y(), v.z() );
	}

	// Convert Cinder quaternion to Bullet quaternion
	btQuaternion toBulletQuaternion( const Quatf& q )
	{
		return btQuaternion( q.v.x, q.v.y, q.v.z, q.w );
	}

	btTransform	toBulletTransform( const ci::Matrix44f &m )
	{
		btTransform trans;
		trans.setFromOpenGLMatrix( m.m );
		return trans;
	}

	// Convert Cinder vector to Bullet vector
	btVector3 toBulletVector3( const Vec3f& v )
	{
		return btVector3( v.x, v.y, v.z );
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////

	// Convert Bullet matrix to Cinder matrix for rigid bodies
	Matrix44f Utilities::getWorldTransform( const btRigidBody* body )
	{
		btTransform trans;
		body->getMotionState()->getWorldTransform( trans );
		Matrix44f matrix;
		trans.getOpenGLMatrix( matrix.m );
		return matrix;
	}

	// Convert Bullet matrix to Cinder matrix for soft bodies
	Matrix44f Utilities::getWorldTransform( const btSoftBody* body )
	{
		btTransform trans = body->getWorldTransform();
		Matrix44f matrix;
		trans.getOpenGLMatrix( matrix.m );
		return matrix;
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////

	// Create terrain from color channel
	btHeightfieldTerrainShape* Utilities::createHeightfieldTerrainShape( const Surface32f& heightField, int32_t stickWidth, int32_t stickLength, 
		float heightScale, float minHeight, float maxHeight, int32_t upAxis, const Vec3f& scale )
	{

		// Create height field shape from channel data
		Channel32f channel( heightField );
		btHeightfieldTerrainShape* shape = new btHeightfieldTerrainShape( stickWidth, stickLength, channel.getData(), heightScale, minHeight, maxHeight, upAxis, PHY_FLOAT, false );
		
		// Scale and return shape
		shape->setLocalScaling( toBulletVector3( scale ) );
		return shape;

	}

	// Creates a concave Bullet mesh from a list of vertices and indices
	btBvhTriangleMeshShape* Utilities::createConcaveMeshShape( const vector<Vec3f>& vertices, const vector<uint32_t>& indices, const Vec3f& scale, float margin )
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
	btConvexHullShape* Utilities::createConvexHullShape( const vector<Vec3f>& vertices, const Vec3f& scale )
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
