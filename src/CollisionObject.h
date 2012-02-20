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
#include "BulletUtils.h"
#include "TetraCube.h"

namespace bullet
{

	/*! Collision object base class */
	class CollisionObject 
	{

	public:

		// Con/de-structor
		explicit CollisionObject() {}
		virtual ~CollisionObject() {}

		// Extend this to draw
		virtual void draw(bool wireframe = false) const = 0;

		// Getters
		virtual int32_t getId() = 0;
		virtual int32_t getId() const = 0;
		virtual double getLifespan() = 0;
		virtual double getLifespan() const = 0;
		virtual double getLifetime() = 0;
		virtual double getLifetime() const = 0;
		virtual ci::Vec3f getPosition() = 0;
		virtual ci::Vec3f getPosition() const = 0;
		virtual ci::Quatf getRotation() = 0;
		virtual ci::Quatf getRotation() const = 0;
		virtual ci::gl::VboMesh getVboMesh() = 0;
		virtual ci::gl::VboMesh getVboMesh() const = 0;
		virtual btDynamicsWorld * getWorld() = 0;
		virtual btDynamicsWorld * getWorld() const = 0;

		// Setters
		virtual void setId(int32_t id) = 0;
		virtual void setLifespan(double time) = 0;

		// Runs update logic
		virtual void update(double step) = 0;

	};

	// Aliases
	typedef std::shared_ptr<CollisionObject> CollisionObjectRef;
	typedef std::vector<CollisionObjectRef> CollisionObjectRefList;

}
