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

#include "SoftBody.h"

#include "bullet/BulletSoftBody/btSoftBodyHelpers.h"

namespace bullet {
	using namespace ci;
	using namespace std;

	btSoftBody* SoftBody::createSoftCloth( btSoftBodyWorldInfo &info, const Vec2f &size, const Vec2i &resolution, 
		int32_t corners, const Vec3f &position, const Quatf &rotation )
	{
		Matrix44f transform;
		transform.setToIdentity();
		transform.translate( position );
		transform.rotate( rotation.v );
		transform.translate( position * -1.0f );
		transform.translate( position );

		float h = size.y * 0.5f;
		float w = size.x * 0.5f;
		btSoftBody* body = btSoftBodyHelpers::CreatePatch(
			info,
			toBulletVector3( transform.transformPoint( Vec3f( -w, 0.0f, -h ) ) ), 
			toBulletVector3( transform.transformPoint( Vec3f(  w, 0.0f, -h ) ) ), 
			toBulletVector3( transform.transformPoint( Vec3f( -w, 0.0f,  h ) ) ), 
			toBulletVector3( transform.transformPoint( Vec3f(  w, 0.0f,  h ) ) ),
			resolution.x, resolution.y, 
			corners, true
			);

		return body;
	}

	btSoftBody*	SoftBody::createSoftMesh( btSoftBodyWorldInfo &info, const TriMesh &mesh, const Vec3f &scale, 
		const Vec3f &position, const Quatf &rotation )
	{
		btScalar* positions	= new btScalar[ mesh.getNumVertices() * 3 ];
		size_t i = 0;
		for ( vector<Vec3f>::const_iterator iter = mesh.getVertices().begin(); iter != mesh.getVertices().end(); ++iter, i += 3 ) {
			positions[ i + 0 ] = iter->x;
			positions[ i + 1 ] = iter->y;
			positions[ i + 2 ] = iter->z;
		}
		
		int* indices		= new int[ mesh.getIndices().size() ];
		i = 0;
		for ( vector<size_t>::const_iterator iter = mesh.getIndices().begin(); iter != mesh.getIndices().end(); ++iter, ++i ) {
			indices[ i ] = (int)*iter;
		}
		
 		btSoftBody* body = btSoftBodyHelpers::CreateFromTriMesh( info, positions, indices, mesh.getNumTriangles() );

		Matrix44f transform;
		transform.setToIdentity();
		transform.translate( position );
		transform.rotate( rotation.v );
		transform.translate( position * -1.0f );
		transform.translate( position );

		body->transform( toBulletTransform( transform ) );
		body->scale( btVector3( scale.x, scale.y, scale.z ) );

		delete [] indices;
		delete [] positions;

		return body;
	}

	SoftCloth::SoftCloth( btSoftBodyWorldInfo &info, const Vec2f &size, const Vec2i &resolution, int32_t corners, 
			const Vec3f &position, const Quatf &rotation ) 
		: CollisionObject()
	{
		mSoftBody	= createSoftCloth( info, size, resolution, corners, position, rotation );
		mScale		= Vec3f::one();

		Vec3f normal	= Vec3f::zero();
		Vec2f offset	= size * 0.5f;
		size_t count	= mSoftBody->m_faces.size();
		for ( size_t i = 0; i < count; ++i ) {
			const btSoftBody::Face&	face = mSoftBody->m_faces[ i ];
			
			Vec3f vert0		= fromBulletVector3( face.m_n[ 0 ]->m_x );
			Vec3f vert1		= fromBulletVector3( face.m_n[ 1 ]->m_x );
			Vec3f vert2		= fromBulletVector3( face.m_n[ 2 ]->m_x );

			Vec2f texCoord0	= ( vert0.xz() + offset ) / size;
			Vec2f texCoord1	= ( vert1.xz() + offset ) / size;
			Vec2f texCoord2	= ( vert2.xz() + offset ) / size;

			mIndices.push_back( i * 3 + 0 );
			mIndices.push_back( i * 3 + 1 );
			mIndices.push_back( i * 3 + 2 );
			
			mTexCoords.push_back( texCoord0 );
			mTexCoords.push_back( texCoord1 );
			mTexCoords.push_back( texCoord2 );
		}

		update();
	}

	SoftMesh::SoftMesh( btSoftBodyWorldInfo &info, const TriMesh &mesh, const Vec3f &scale, const Vec3f &position, const Quatf &rotation )
		: CollisionObject()
	{
		mSoftBody	= createSoftMesh( info, mesh, scale, position, rotation );
		mScale		= scale;

		mIndices	= mesh.getIndices();
		mNormals	= mesh.getNormals();
		mPositions	= mesh.getVertices();
		mTexCoords	= mesh.getTexCoords();

		update();
	}
}
