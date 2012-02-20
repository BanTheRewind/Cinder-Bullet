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

#pragma once

// Includes
#include "CollisionObject.h"
#include "DynamicsWorld.h"
#include "BulletUtils.h"

namespace bullet
{

	/*! Soft object base class */
	class SoftObject : public CollisionObject
	{

	public:

		// Destructor
		~SoftObject();
		
		// Draw
		void draw(bool wireframe) const;

		// Runs update logic
		void update(double step);

		// Getters
		btSoftBody * getBody() { return mBody; }
		btSoftBody * getBody() const { return mBody; }
		int32_t getId() { return mId; }
		int32_t getId() const { return mId; }
		btSoftBodyWorldInfo * getInfo() { return mInfo; }
		btSoftBodyWorldInfo * getInfo() const { return mInfo; }
		double getLifespan() { return mLifespan; }
		double getLifespan() const { return mLifespan; }
		double getLifetime() { return mLifetime; }
		double getLifetime() const { return mLifetime; }
		ci::Vec3f getPosition() { return mPosition; }
		ci::Vec3f getPosition() const { return mPosition; }
		ci::Quatf getRotation() { return mRotation; }
		ci::Quatf getRotation() const { return mRotation; }
		ci::gl::VboMesh getVboMesh() { return mVboMesh; }
		ci::gl::VboMesh getVboMesh() const { return mVboMesh; }
		btDynamicsWorld * getWorld() { return mWorld; }
		btDynamicsWorld * getWorld() const { return mWorld; }

		// Setters
		void setId(int32_t id) { mId = id; }
		void setLifespan(double time) { mLifespan = time; }

	protected:

		// Con/de-structor
		SoftObject(DynamicsWorldRef world, const ci::Vec3f & position = ci::Vec3f::zero(), const ci::Quatf & rotation = ci::Quatf());
		
		// Properties
		btSoftBody * mBody;
		int32_t mId;
		btSoftBodyWorldInfo * mInfo;
		double mLifespan;
		double mLifetime;
		ci::Vec3f mPosition;
		ci::Quatf mRotation;
		ci::gl::VboMesh::Layout mVboLayout;
		ci::gl::VboMesh mVboMesh;
		btDynamicsWorld * mWorld;

	};

	// Aliases
	typedef std::shared_ptr<SoftObject> SoftObjectRef;
	typedef std::vector<SoftObjectRef> SoftObjectRefList;

	/*! Soft box */
	class SoftBox : public SoftObject 
	{
	public:
		static SoftObjectRef create(DynamicsWorldRef world, const ci::Vec3f & dimensions = ci::Vec3f(10.0f, 10.0f, 10.0f), const ci::Vec3f & position = ci::Vec3f::zero(), const ci::Quatf & rotation = ci::Quatf());
	private:
		SoftBox(DynamicsWorldRef world, const ci::Vec3f & dimensions = ci::Vec3f(10.0f, 10.0f, 10.0f), const ci::Vec3f & position = ci::Vec3f::zero(), const ci::Quatf & rotation = ci::Quatf());
	};

	/*! Soft sphere */
	class SoftSphere : public SoftObject 
	{
	public:
		static SoftObjectRef create(DynamicsWorldRef world, float radius = 10.0f, int32_t segments = 16, const ci::Vec3f & position = ci::Vec3f::zero(), const ci::Quatf & rotation = ci::Quatf());
	private:
		SoftSphere(DynamicsWorldRef world, float radius = 10.0f, int32_t segments = 16, const ci::Vec3f & position = ci::Vec3f::zero(), const ci::Quatf & rotation = ci::Quatf());
	};

	/*! Soft tetra cube */
	class SoftTetraBox : public SoftObject 
	{
	public:
		static SoftObjectRef create(DynamicsWorldRef world, const ci::Vec3f & dimensions = ci::Vec3f(10.0f, 10.0f, 10.0f), const ci::Vec3f & position = ci::Vec3f::zero(), const ci::Quatf & rotation = ci::Quatf());
	private:
		SoftTetraBox(DynamicsWorldRef world, const ci::Vec3f & dimensions = ci::Vec3f(10.0f, 10.0f, 10.0f), const ci::Vec3f & position = ci::Vec3f::zero(), const ci::Quatf & rotation = ci::Quatf());
		TetraCube mTetraCube;
	};

	// These methods return the object as the base type for 
	// compatibility with the dynamics world class
	CollisionObjectRef createSoftBox(DynamicsWorldRef world, const ci::Vec3f & dimensions = ci::Vec3f(10.0f, 10.0f, 10.0f), const ci::Vec3f & position = ci::Vec3f::zero(), const ci::Quatf & rotation = ci::Quatf());
	CollisionObjectRef createSoftSphere(DynamicsWorldRef world, float radius = 10.0f, int32_t segments = 16, const ci::Vec3f & position = ci::Vec3f::zero(), const ci::Quatf & rotation = ci::Quatf());
	CollisionObjectRef createSoftTetraBox(DynamicsWorldRef world, const ci::Vec3f & dimensions = ci::Vec3f(10.0f, 10.0f, 10.0f), const ci::Vec3f & position = ci::Vec3f::zero(), const ci::Quatf & rotation = ci::Quatf());

}
