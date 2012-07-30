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
#include "SoftBody.h"


#include "bullet/BulletSoftBody//btSoftBodyHelpers.h"

namespace bullet
{

	// Import namespaces
	using namespace ci;
	using namespace std;

	// Creates soft cloth
	btSoftBody* SoftBody::createSoftCloth( btSoftBodyWorldInfo &info, const Vec2f &size, const Vec2i &resolution, 
		int32_t corners, const Vec3f &position, const Quatf &rotation )
	{

		// Use a matrix to position corners
		Matrix44f transform;
		transform.setToIdentity();
		transform.translate( position );
		transform.rotate( rotation.v );
		transform.translate( position * -1.0f );
		transform.translate( position );

		// Create and return soft body
		float h = size.y * 0.5f;
		float w = size.x * 0.5f;
		btSoftBody*		body = btSoftBodyHelpers::CreatePatch(
			info,
			toBulletVector3( transform.transformPoint( Vec3f( -w, h, -w ) ) ), 
			toBulletVector3( transform.transformPoint( Vec3f(  w, h, -w ) ) ), 
			toBulletVector3( transform.transformPoint( Vec3f( -w, h,  w ) ) ), 
			toBulletVector3( transform.transformPoint( Vec3f(  w, h,  w ) ) ),
			resolution.x, resolution.y, 
			corners, true
			);
		return body;

	}

	SoftCloth::SoftCloth( btSoftBodyWorldInfo &info, const Vec2f &size, const Vec2i &resolution, int32_t corners, 
			const Vec3f &position, const Quatf &rotation ) 
		: CollisionObject( position, rotation )
	{

		// Create body
		mSoftBody = createSoftCloth( info, size, resolution, corners, position, rotation );

		// Set scale
		mScale = Vec3f( size, 1.0f );

		// Iterate through dimensions to set vertex data
		float halfHeight	= size.y * 0.5f;
		float halfWidth		= size.x * 0.5f;
		int32_t height		= resolution.y;
		int32_t width		= resolution.x;
		Vec2f delta			= size / Vec2f( resolution );
		for ( int32_t y = 0; y < height; y++ ) {
			for ( int32_t x = 0; x < width; x++ ) {

				// Add texture coordinate
				mTexCoords.push_back( Vec2f( (float)x / (float)width, (float)y / (float)height ) );

				// Add indices for this quad
				int32_t xn = x + 1 >= width ? 0 : 1;
				int32_t yn = y + 1 >= height ? 0 : 1;
				mIndices.push_back( x + height * y );
				mIndices.push_back( ( x + xn ) + height * y);
				mIndices.push_back( ( x + xn ) + height * ( y + yn ) );
				mIndices.push_back( x + height * ( y + yn ) );
				mIndices.push_back( ( x + xn ) + height * ( y + yn ) );
				mIndices.push_back( x + height * y );

				Vec3f position( (float)x * delta.x - halfWidth, 0.0f, (float)y * delta.y - halfHeight );
				mPositions.push_back( position );

				mNormals.push_back( Vec3f::zero() );

			}

		}

		// Loop through again to set normals
		for ( int32_t y = 0; y < height - 1; y++ ) {
			for (int32_t x = 0; x < width - 1; x++) {

				// Get vertices of this triangle
				Vec3f vert0 = mPositions[ mIndices[ ( x + height * y ) * 6 ] ];
				Vec3f vert1 = mPositions[ mIndices[ ( ( x + 1 ) + height * y ) * 6 ] ];
				Vec3f vert2 = mPositions[ mIndices[ ( ( x + 1 ) + height * ( y + 1 ) ) * 6 ] ];

				// Calculate normal
				mNormals[ x + height * y ] = Vec3f( ( vert1 - vert0 ).cross( vert1 - vert2 ).normalized() );

			}
		}

		// Set VBO
		mVboMesh = VboMeshManager::create( mIndices, mPositions, mNormals, mTexCoords );

	}

}
