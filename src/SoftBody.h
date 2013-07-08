/*
* CinderBullet originally created by Peter Holzkorn on 2/16/10
* 
* Copyright (c) 2013, Ban the Rewind
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

#include "CollisionObject.h"
#include "DynamicsWorld.h"
#include "Utilities.h"

namespace bullet {
	class DynamicsWorld;

	class SoftBody
	{
	protected:
		static btSoftBody*	createSoftCloth( btSoftBodyWorldInfo &info, const ci::Vec2f &size, const ci::Vec2i &resolution, 
			int32_t corners, const ci::Vec3f &position, const ci::Quatf &rotation );
		static btSoftBody*	createSoftHull( btSoftBodyWorldInfo &info, const ci::TriMesh &mesh, const ci::Vec3f &scale, 
			const ci::Vec3f &position, const ci::Quatf &rotation );
		static btSoftBody*	createSoftMesh( btSoftBodyWorldInfo &info, const ci::TriMesh &mesh, const ci::Vec3f &scale, 
			const ci::Vec3f &position, const ci::Quatf &rotation );
	};

	class SoftCloth : public CollisionObject, public SoftBody
	{
	public:
		static const int32_t	CLOTH_ATTACH_CORNER_NONE	= 0;
		static const int32_t	CLOTH_ATTACH_CORNER_0		= 1;
		static const int32_t	CLOTH_ATTACH_CORNER_1		= 2; 
		static const int32_t	CLOTH_ATTACH_CORNER_2		= 4; 
		static const int32_t	CLOTH_ATTACH_CORNER_3		= 8; 
		static const int32_t	CLOTH_ATTACH_CORNER_ALL		= 15;
	protected:
		SoftCloth( btSoftBodyWorldInfo &info, const ci::Vec2f &size, const ci::Vec2i &resolution, int32_t corners, 
			const ci::Vec3f &position, const ci::Quatf &rotation );
		friend CollisionObjectRef	createSoftCloth( const DynamicsWorldRef &world, const ci::Vec2f &size, 
			const ci::Vec2i &resolution, int32_t corners, const ci::Vec3f &position, const ci::Quatf &rotation );
	};

	class SoftHull : public CollisionObject, public SoftBody
	{
	protected:
		SoftHull( btSoftBodyWorldInfo &info, const ci::TriMesh &mesh, const ci::Vec3f &scale, 
			const ci::Vec3f &position, const ci::Quatf &rotation );
		friend CollisionObjectRef	createSoftHull( const DynamicsWorldRef &world, const ci::TriMesh &mesh, const ci::Vec3f &scale, 
													const ci::Vec3f &position, const ci::Quatf &rotation );
	};

	class SoftMesh : public CollisionObject, public SoftBody
	{
	protected:
		SoftMesh( btSoftBodyWorldInfo &info, const ci::TriMesh &mesh, const ci::Vec3f &scale, 
			const ci::Vec3f &position, const ci::Quatf &rotation );
		friend CollisionObjectRef	createSoftMesh( const DynamicsWorldRef &world, const ci::TriMesh &mesh, const ci::Vec3f &scale, 
													const ci::Vec3f &position, const ci::Quatf &rotation );
	};
}
