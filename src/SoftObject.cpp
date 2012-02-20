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

	// Constructor
	SoftObject::SoftObject(DynamicsWorldRef world, const Vec3f & position, const Quatf & rotation) 
	{

		// Initialize VBO layout
		mVboLayout.setStaticIndices();
		mVboLayout.setDynamicNormals();
		mVboLayout.setDynamicPositions();

		// Define properties
		mInfo = & (world->getInfo());
		mRotation = rotation;
		mWorld = world->getWorld();
		mLifespan = 0.0;
		mLifetime = 0.0;

	}

	// Destructor
	SoftObject::~SoftObject() 
	{

		// Clean up
		if (mBody)
		{
			((btSoftRigidDynamicsWorld *)mWorld)->removeSoftBody(mBody);
			delete mBody;
		}

	}

	// Draw
	void SoftObject::draw(bool wireframe) const
	{

		// Draw VBO mesh
		gl::pushMatrices();
		if (wireframe)
			gl::enableWireframe();
		glMultMatrixf(bullet::getWorldTransform(mBody).m);
		gl::draw(mVboMesh);
		if (wireframe)
			gl::disableWireframe();
		gl::popMatrices();

	}

	// Runs update logic
	void SoftObject::update(double step)
	{

		// Add time step to lifetime
		mLifetime += step;
		
		// Update position and rotation
		mPosition = bullet::fromBulletVector3(mBody->m_bounds[0].lerp(mBody->m_bounds[1], 0.5f));
		mRotation = Quatf(bullet::getWorldTransform(mBody));

		// TO DO update VBO mesh

	}

	/****** CONSTRUCTORS ******/

	// Soft box
	SoftBox::SoftBox(DynamicsWorldRef world, const Vec3f & dimensions, const Vec3f & position, const Quatf & rotation) 
		: SoftObject(world, position, rotation)
	{

		// Set object dimensions
		mBody = bullet::create(world->getWorld(), world->getInfo(), dimensions, position, rotation);

		// TO DO create VBO mesh

	}

	// Soft sphere
	SoftSphere::SoftSphere(DynamicsWorldRef world, float radius, int32_t segments, const Vec3f & position, const Quatf & rotation) 
		: SoftObject(world, position, rotation)
	{

		// Set object dimensions
		mBody = bullet::create(world->getWorld(), world->getInfo(), radius, segments, position, rotation);

		// TO DO create VBO mesh

	}

	// Soft tetra cube
	SoftTetraBox::SoftTetraBox(DynamicsWorldRef world, const Vec3f & dimensions, const Vec3f & position, const Quatf & rotation) 
		: SoftObject(world, position, rotation)
	{

		// Set object dimensions
		mBody = bullet::create(world->getWorld(), world->getInfo(), mTetraCube, dimensions, position, rotation);

		// TO DO create VBO mesh

	}

	/****** Methods for creating object pointers ******/

	// Box
	SoftObjectRef SoftBox::create(DynamicsWorldRef world, const Vec3f & dimensions, const Vec3f & position, const Quatf & rotation)
	{
		return SoftObjectRef(new SoftBox(world, dimensions, position, rotation));
	}
	CollisionObjectRef createSoftBox(DynamicsWorldRef world, const Vec3f & dimensions, const Vec3f & position, const Quatf & rotation)
	{
		return (CollisionObjectRef)SoftBox::create(world, dimensions, position, rotation);
	}

	// Sphere
	SoftObjectRef SoftSphere::create(DynamicsWorldRef world, float radius, int32_t segments, const Vec3f & position, const Quatf & rotation)
	{
		return SoftObjectRef(new SoftSphere(world, radius, segments, position, rotation));
	}
	CollisionObjectRef createSoftSoftSphere(DynamicsWorldRef world, float radius, int32_t segments, const Vec3f & position, const Quatf & rotation)
	{
		return (CollisionObjectRef)SoftSphere::create(world, radius, segments, position, rotation);
	}

	// Tetra box
	SoftObjectRef SoftTetraBox::create(DynamicsWorldRef world, const Vec3f & dimensions, const Vec3f & position, const Quatf & rotation)
	{
		return SoftObjectRef(new SoftTetraBox(world, dimensions, position, rotation));
	}
	CollisionObjectRef createSoftTetraBox(DynamicsWorldRef world, const Vec3f & dimensions, const Vec3f & position, const Quatf & rotation)
	{
		return (CollisionObjectRef)SoftTetraBox::create(world, dimensions, position, rotation);
	}

}
