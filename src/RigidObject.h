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

	/*! Rigid object base class */
	class RigidObject : public CollisionObject
	{

	public:

		// Destructor
		~RigidObject();

		// Draw
		void draw(bool wireframe) const;

		// Runs update logic
		void update(double step);

		// Getters
		btRigidBody * getBody() { return mBody; }
		btRigidBody * getBody() const { return mBody; }
		int32_t getId() { return mId; }
		int32_t getId() const { return mId; }
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
		RigidObject(DynamicsWorldRef world, const ci::Vec3f & position = ci::Vec3f::zero(), const ci::Quatf & rotation = ci::Quatf());
		
		// Properties
		btRigidBody * mBody;
		int32_t mId;
		double mLifespan;
		double mLifetime;
		ci::Vec3f mPosition;
		ci::Quatf mRotation;
		std::vector<uint32_t> mVboIndices;
		ci::gl::VboMesh::Layout mVboLayout;
		ci::gl::VboMesh mVboMesh;
		std::vector<ci::Vec3f> mVboNormals;
		std::vector<ci::Vec3f> mVboPositions;
		btDynamicsWorld * mWorld;

		// Set, clear VBO data
		void setVboData(GLenum primitiveType = GL_TRIANGLES);
		void clearVboData();

	};

	// Aliases
	typedef std::shared_ptr<RigidObject> RigidObjectRef;
	typedef std::vector<RigidObjectRef> RigidObjectRefList;

	/*! Rigid box */
	class RigidBox : public RigidObject 
	{
	public:
		static RigidObjectRef create(DynamicsWorldRef world, const ci::Vec3f & dimensions = ci::Vec3f(10.0f, 10.0f, 10.0f), const ci::Vec3f & position = ci::Vec3f::zero(), const ci::Quatf & rotation = ci::Quatf());
	private:
		RigidBox(DynamicsWorldRef world, const ci::Vec3f & dimensions = ci::Vec3f(10.0f, 10.0f, 10.0f), const ci::Vec3f & position = ci::Vec3f::zero(), const ci::Quatf & rotation = ci::Quatf());
	};

	/*! Rigid cylinder */
	class RigidCylinder : public RigidObject 
	{
	public:
		static RigidObjectRef create(DynamicsWorldRef world, float topRadius = 10.0f, float bottomRadius = 10.0f, float height = 20.0f, int32_t segments = 16, const ci::Vec3f & position = ci::Vec3f::zero(), const ci::Quatf & rotation = ci::Quatf());
	private:
		RigidCylinder(DynamicsWorldRef world, float topRadius = 10.0f, float bottomRadius = 10.0f, float height = 20.0f, int32_t segments = 16, const ci::Vec3f & position = ci::Vec3f::zero(), const ci::Quatf & rotation = ci::Quatf());
	};

	/*! Rigid hull */
	class RigidHull : public RigidObject 
	{
	public:
		static RigidObjectRef create(DynamicsWorldRef world, const ci::TriMesh & mesh, const ci::Vec3f & scale = ci::Vec3f(1.0f, 1.0f, 1.0f), const ci::Vec3f & position = ci::Vec3f::zero(), const ci::Quatf & rotation = ci::Quatf());
	private:
		RigidHull(DynamicsWorldRef world, const ci::TriMesh & mesh, const ci::Vec3f & scale = ci::Vec3f(1.0f, 1.0f, 1.0f), const ci::Vec3f & position = ci::Vec3f::zero(), const ci::Quatf & rotation = ci::Quatf());
	};

	// Rigid mesh */
	class RigidMesh : public RigidObject 
	{
	public:
		static RigidObjectRef create(DynamicsWorldRef world, const ci::TriMesh & mesh, const ci::Vec3f & scale = ci::Vec3f(1.0f, 1.0f, 1.0f), float margin = 0.05f, const ci::Vec3f & position = ci::Vec3f::zero(), const ci::Quatf & rotation = ci::Quatf());
	private:
		RigidMesh(DynamicsWorldRef world, const ci::TriMesh & mesh, const ci::Vec3f & scale = ci::Vec3f(1.0f, 1.0f, 1.0f), float margin = 0.05f, const ci::Vec3f & position = ci::Vec3f::zero(), const ci::Quatf & rotation = ci::Quatf());
	};

	/*! Rigid sphere */
	class RigidSphere : public RigidObject 
	{
	public:
		static RigidObjectRef create(DynamicsWorldRef world, float radius = 10.0f, int32_t segments = 16, const ci::Vec3f & position = ci::Vec3f::zero(), const ci::Quatf & rotation = ci::Quatf());
	private:
		RigidSphere(DynamicsWorldRef world, float radius = 10.0f, int32_t segments = 16, const ci::Vec3f & position = ci::Vec3f::zero(), const ci::Quatf & rotation = ci::Quatf());
	};

	/*! Rigid terrain */
	class RigidTerrain : public RigidObject 
	{
	public:
		static RigidObjectRef create(DynamicsWorldRef world, const ci::Surface32f & heightField, int32_t stickWidth, int32_t stickLength, float minHeight = -200.0f, float maxHeight = 200.0f, int32_t upAxis = 1, const ci::Vec3f & scale = ci::Vec3f(1.0f, 100.0f, 1.0f), const ci::Vec3f & position = ci::Vec3f::zero(), const ci::Quatf & rotation = ci::Quatf());
	private:
		RigidTerrain(DynamicsWorldRef world, const ci::Surface32f & heightField, int32_t stickWidth, int32_t stickLength, float minHeight = -200.0f, float maxHeight = 200.0f, int32_t upAxis = 1, const ci::Vec3f & scale = ci::Vec3f(1.0f, 100.0f, 1.0f), const ci::Vec3f & position = ci::Vec3f::zero(), const ci::Quatf & rotation = ci::Quatf());
	};

	/*! Rigid torus */
	/*class RigidTorus : public RigidObject 
	{
	public:
		static CollisionObjectRef create(DynamicsWorldRef world, float innerRadius = 5.0f, float outerRadius = 10.0f, int32_t segments = 16, const ci::Vec3f & position = ci::Vec3f::zero(), const ci::Quatf & rotation = ci::Quatf());
	private:
		RigidTorus(DynamicsWorldRef world, float innerRadius = 5.0f, float outerRadius = 10.0f, int32_t segments = 16, const ci::Vec3f & position = ci::Vec3f::zero(), const ci::Quatf & rotation = ci::Quatf());
	};*/

	// These methods return the object as the base type for 
	// compatibility with the dynamics world class
	CollisionObjectRef createRigidBox(DynamicsWorldRef world, const ci::Vec3f & dimensions = ci::Vec3f(10.0f, 10.0f, 10.0f), const ci::Vec3f & position = ci::Vec3f::zero(), const ci::Quatf & rotation = ci::Quatf());
	CollisionObjectRef createRigidCylinder(DynamicsWorldRef world, float topRadius = 10.0f, float bottomRadius = 10.0f, float height = 20.0f, int32_t segments = 16, const ci::Vec3f & position = ci::Vec3f::zero(), const ci::Quatf & rotation = ci::Quatf());
	CollisionObjectRef createRigidHull(DynamicsWorldRef world, const ci::TriMesh & mesh, const ci::Vec3f & scale = ci::Vec3f(1.0f, 1.0f, 1.0f), const ci::Vec3f & position = ci::Vec3f::zero(), const ci::Quatf & rotation = ci::Quatf());
	CollisionObjectRef createRigidMesh(DynamicsWorldRef world, const ci::TriMesh & mesh, const ci::Vec3f & scale = ci::Vec3f(1.0f, 1.0f, 1.0f), float margin = 0.05f, const ci::Vec3f & position = ci::Vec3f::zero(), const ci::Quatf & rotation = ci::Quatf());
	CollisionObjectRef createRigidSphere(DynamicsWorldRef world, float radius = 10.0f, int32_t segments = 16, const ci::Vec3f & position = ci::Vec3f::zero(), const ci::Quatf & rotation = ci::Quatf());
	CollisionObjectRef createRigidTerrain(DynamicsWorldRef world, const ci::Surface32f & heightField, int32_t stickWidth, int32_t stickLength, float minHeight = -200.0f, float maxHeight = 200.0f, int32_t upAxis = 1, const ci::Vec3f & scale = ci::Vec3f(1.0f, 100.0f, 1.0f), const ci::Vec3f & position = ci::Vec3f::zero(), const ci::Quatf & rotation = ci::Quatf());
	//CollisionObjectRef createRigidTorus(DynamicsWorldRef world, float innerRadius = 5.0f, float outerRadius = 10.0f, int32_t segments = 16, const ci::Vec3f & position = ci::Vec3f::zero(), const ci::Quatf & rotation = ci::Quatf());

}
