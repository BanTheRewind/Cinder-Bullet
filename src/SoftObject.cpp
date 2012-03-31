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
#include "SoftObject.h"

namespace bullet
{

	// Import namespaces
	using namespace ci;
	using namespace std;

	// Create soft body from arbitrary collision shape
	/*btSoftBody* SoftBody::create( btDynamicsWorld* world, btSoftBodyWorldInfo& info, btCollisionShape* shape, const Vec3f& position, const Quatf& rotation )
	{

		// Create soft body
		btSoftBody* body = new btSoftBody( &info );
		btTransform transform;
		transform.setIdentity();
		transform.setOrigin( bullet::toBulletVector3( position ) );
		body->transform( transform );

		// Add and return body
		( (btSoftRigidDynamicsWorld*)world )->addSoftBody( body );
		return body;

	}

	// Create soft mesh
	btSoftBody* SoftBody::create( btDynamicsWorld* world, btSoftBodyWorldInfo& info, btConvexHullShape* shape, const Vec3f& position, const Quatf& rotation, float mass )
	{

		// Set vertex positions
		btVector3 vertices;
		shape->getVertex( shape->getNumPoints(), vertices );

		// Create soft body
		btSoftBody* body = btSoftBodyHelpers::CreateFromConvexHull( info,& vertices, shape->getNumPoints(), false );

		// Set mass and position
		btTransform transform;
		transform.setIdentity();
		transform.setOrigin( bullet::toBulletVector3( position ) );
		body->transform( transform );
		body->setTotalMass( mass, true );

		// Add and return soft body
		( (btSoftRigidDynamicsWorld*)world )->addSoftBody( body );
		return body;

	}*/

	// Create soft box
	/*btSoftBody* SoftBody::create( btDynamicsWorld* world, btSoftBodyWorldInfo& info, const Vec3f& dimensions, const Vec3f& position, const Quatf& rotation )
	{

		// Define cube
		const btVector3	hull = bullet::toBulletVector3( dimensions ) * 0.5f;
		const btVector3	cube [] = {
			hull* btVector3( -1.0f, -1.0f, -1.0f ), 
			hull* btVector3( +1.0f, -1.0f, -1.0f ), 
			hull* btVector3( -1.0f, +1.0f, -1.0f ), 
			hull* btVector3( +1.0f, +1.0f, -1.0f ), 
			hull* btVector3( -1.0f, -1.0f, +1.0f ), 
			hull* btVector3( +1.0f, -1.0f, +1.0f ), 
			hull* btVector3( -1.0f, +1.0f, +1.0f ), 
			hull* btVector3( +1.0f, +1.0f, +1.0f )
		};

		// Create soft cube
		btSoftBody* body = btSoftBodyHelpers::CreateFromConvexHull( info, cube, 8 );

		// Set position, rotation, and mass
		btTransform transform;
		transform.setIdentity();
		transform.setRotation( bullet::toBulletQuaternion( rotation ) );
		transform.setOrigin( bullet::toBulletVector3( position ) );
		body->transform( transform );
		body->setTotalMass( ( dimensions.x * 0.5f ) * ( dimensions.y * 0.5f ) * ( dimensions.z * 0.5f ) * (float)M_PI * 4.0f / 3.0f, true );

		// Add and return soft body
		( (btSoftRigidDynamicsWorld*)world )->addSoftBody( body );
		return body;

	}

	// Creates a soft sphere
	btSoftBody* SoftBody::create( btDynamicsWorld* world, btSoftBodyWorldInfo& info, float radius, int32_t segments, const Vec3f& position, const Quatf& rotation )
	{

		// Create soft sphere
		btSoftBody* body = btSoftBodyHelpers::CreateEllipsoid( info, btVector3( 0.0f, 0.0f, 0.0f ), btVector3( radius, radius, radius ), segments );

		// Set position, rotation, and mass
		btTransform transform;
		transform.setIdentity();
		transform.setRotation( bullet::toBulletQuaternion( rotation ) );
		transform.setOrigin( bullet::toBulletVector3( position ) );
		body->transform( transform );
		body->setTotalMass( radius * radius * radius * (float)M_PI * 4.0f / 3.0f, true );

		// Add and return soft body
		( (btSoftRigidDynamicsWorld*)world )->addSoftBody( body );
		return body;

	}

	// Create soft tetra box
	btSoftBody* SoftBody::create( btDynamicsWorld* world, btSoftBodyWorldInfo& info, const TetraCube& tetraCube, 
		const Vec3f& dimensions, const Vec3f& position, const Quatf& rotation )
	{

		// Define cube
		const btVector3	hull = bullet::toBulletVector3( dimensions )* 0.5f;
		const btVector3	cube [] = 
		{
			hull* btVector3( -1.0f, -1.0f, -1.0f ), 
			hull* btVector3( +1.0f, -1.0f, -1.0f ), 
			hull* btVector3( -1.0f, +1.0f, -1.0f ), 
			hull* btVector3( +1.0f, +1.0f, -1.0f ), 
			hull* btVector3( -1.0f, -1.0f, +1.0f ), 
			hull* btVector3( +1.0f, -1.0f, +1.0f ), 
			hull* btVector3( -1.0f, +1.0f, +1.0f ), 
			hull* btVector3( +1.0f, +1.0f, +1.0f )
		};

		// Create soft cube
		btSoftBody* body = btSoftBodyHelpers::CreateFromConvexHull( info, cube, 8 );

		// Set position, rotation, and mass
		btTransform transform;
		transform.setIdentity();
		transform.setRotation( bullet::toBulletQuaternion( rotation ) );
		transform.setOrigin( bullet::toBulletVector3( position ) );
		body->transform( transform );
		body->setTotalMass( ( dimensions.x * 0.5f ) * ( dimensions.y * 0.5f )* ( dimensions.z * 0.5f ) * (float)M_PI * 4.0f / 3.0f, true );
			
		// Add and return soft body
		( (btSoftRigidDynamicsWorld*)world )->addSoftBody( body );
		return body;

	}*/
	/*
	// Create a soft box from AxisAlignedBox3f
	btSoftBody* SoftBody::create( btDynamicsWorld* world, btSoftBodyWorldInfo& info, const AxisAlignedBox3f& box, const Vec3f& position, const Quatf& rotation )
	{

		// Determine box size
		Vec3f min = box.getMin();
		Vec3f max = box.getMax();
		Vec3f size = Vec3f( math<float>::abs( max.x - min.x ), math<float>::abs( max.y - min.y ), math<float>::abs( max.z - min.z ) );

		// Create and return box
		return create( world, info, size, position, rotation );

	}

	// Create a soft sphere from Sphere
	btSoftBody* SoftBody::create( btDynamicsWorld* world, btSoftBodyWorldInfo& info, const Sphere& sphere, int32_t segments, const Vec3f& position, const Quatf& rotation )
	{

		// Create and return soft sphere
		return create( world, info, sphere.getRadius(), segments, position, rotation );		

	}*/

	/****** CONSTRUCTORS ******/

	SoftObject::SoftObject( btDynamicsWorld* world, const Vec3f &position, const Quatf &rotation )
		: CollisionObjectBase( world, position, rotation )
	{
	}

	// Soft box
	/*SoftBox::SoftBox( DynamicsWorldRef world, const Vec3f &dimensions, const Vec3f &position, const Quatf &rotation ) 
		: SoftObject( world, position, rotation )
	{

		// Set object dimensions
		mSoftBody = SoftBody::create( world->getWorld(), world->getInfo(), dimensions, position, rotation );

		// TO DO create VBO mesh

	}

	// Soft sphere
	SoftSphere::SoftSphere( DynamicsWorldRef world, float radius, int32_t segments, const Vec3f &position, const Quatf &rotation ) 
		: SoftObject( world, position, rotation )
	{

		// Set object dimensions
		mSoftBody = SoftBody::create( world->getWorld(), world->getInfo(), radius, segments, position, rotation );

		// TO DO create VBO mesh

	}

	// Soft tetra cube
	SoftTetraBox::SoftTetraBox( DynamicsWorldRef world, const Vec3f &dimensions, const Vec3f &position, const Quatf &rotation ) 
		: SoftObject( world, position, rotation )
	{

		// Set object dimensions
		mSoftBody = SoftBody::create( world->getWorld(), world->getInfo(), mTetraCube, dimensions, position, rotation );

		// TO DO create VBO mesh

	}*/

}
