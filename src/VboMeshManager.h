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

#include "cinder/gl/Vbo.h"
#include "cinder/Vector.h"
#include <map>

namespace bullet
{

	// Texture pointer alias
	typedef std::shared_ptr<ci::gl::VboMesh> VboMeshRef;
	typedef std::weak_ptr<ci::gl::VboMesh> VboMeshWeakRef;

	// Manages VBO meshes
	class VboMeshManager 
	{

	public:

		typedef enum 
		{
			PRIMITIVE_NONE, PRIMITIVE_BOX, PRIMITIVE_CONE, PRIMITIVE_CYLINDER, PRIMITIVE_SPHERE
		} PrimitiveType;

		static VboMeshRef create( PrimitiveType type, const ci::Vec3f &scale = ci::Vec3f::one(), uint32_t segments = 0 );
		static VboMeshRef create( const std::vector<uint32_t> &indices, const std::vector<ci::Vec3f> &positions, 
								  const std::vector<ci::Vec3f> &normals, const std::vector<ci::Vec2f> &texCoords, 
								  GLenum primitiveType = GL_TRIANGLES );

	private:
		
		static VboMeshRef createBox();
		//static VboMeshRef createCone( const ci::Vec3f &scale, uint32_t segments );
		static VboMeshRef createCylinder( const ci::Vec3f &scale, uint32_t segments );
		static VboMeshRef createSphere( uint32_t segments );

		class PrimitiveInfo
		{
		public:
			PrimitiveInfo( PrimitiveType type, const ci::Vec3f &scale, uint32_t segments );
			bool			operator<( const PrimitiveInfo &rhs ) const;
			bool			operator==( const PrimitiveInfo &rhs ) const;
			bool			operator!=( const PrimitiveInfo &rhs ) const;
		private:
			uint32_t		mSegments;
			ci::Vec3f		mScale;
			PrimitiveType	mType;
		};

		struct PrimitiveInfoSort
		{
			bool operator()( const PrimitiveInfo &lhs, const PrimitiveInfo &rhs) const;
		};

		// List of weak pointers to textures
		typedef std::map<PrimitiveInfo, VboMeshWeakRef, PrimitiveInfoSort> VboMeshList;
		static VboMeshList	sVboMeshList;

	};

}
