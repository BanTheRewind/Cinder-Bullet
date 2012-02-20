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
#include "RigidObject.h"

#include <cinder/Utilities.h>

namespace bullet
{

	// Import namespaces
	using namespace ci;
	using namespace std;

	// Constructor
	RigidObject::RigidObject(DynamicsWorldRef world, const Vec3f & position, const Quatf & rotation) 
	{

		// Initialize VBO layout
		mVboLayout.setStaticIndices();
		mVboLayout.setStaticNormals();
		mVboLayout.setStaticPositions();

		// Define properties
		mWorld = world->getWorld();
		mPosition = position;
		mRotation = rotation;
		mLifespan = 0.0;
		mLifetime = 0.0;

	}

	// Internal object destructor
	RigidObject::~RigidObject()
	{

		// Clear vectors (just in case)
		clearVboData();

		// Clean up
		if (mBody)
		{
			mWorld->removeRigidBody(mBody);
			if (mBody->getMotionState())
				delete mBody->getMotionState();
			delete mBody;
		}

	}

	// Clear VBO data
	void RigidObject::clearVboData()
	{

		// Clear VBO vectors
		mVboIndices.clear();
		mVboNormals.clear();
		mVboPositions.clear();

	}

	// Render object
	void RigidObject::draw(bool wireframe) const
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
	void RigidObject::update(double step)
	{

		// Add time step to lifetime
		mLifetime += step;

		// Update position and rotation
		mPosition = bullet::fromBulletVector3(mBody->getCenterOfMassPosition());
		mRotation = Quatf(bullet::getWorldTransform(mBody));

	}

	// Sets VBO data
	void RigidObject::setVboData(GLenum primitiveType)
	{

		// Set VBO data
		mVboMesh = gl::VboMesh(mVboPositions.size(), mVboIndices.size(), mVboLayout, primitiveType);
		mVboMesh.bufferIndices(mVboIndices);
		mVboMesh.bufferNormals(mVboNormals);
		mVboMesh.bufferPositions(mVboPositions);

	}

	/****** CONSTRUCTORS ******/

	// Box
	RigidBox::RigidBox(DynamicsWorldRef world, const Vec3f & dimensions, const Vec3f & position, const Quatf & rotation) 
		: RigidObject(world, position, rotation)
	{

		// Create body
		mBody = bullet::create(world->getWorld(), dimensions, position, rotation);

		// Define vertices
		Vec3f size = dimensions * 0.5f;
		mVboPositions.push_back(size * Vec3f(-1.0f, -1.0f, -1.0f)); // 0 ---
		mVboPositions.push_back(size * Vec3f( 1.0f, -1.0f, -1.0f)); // 1 +--
		mVboPositions.push_back(size * Vec3f(-1.0f,  1.0f, -1.0f)); // 2 -+-
		mVboPositions.push_back(size * Vec3f( 1.0f,  1.0f, -1.0f)); // 3 ++-
		mVboPositions.push_back(size * Vec3f(-1.0f, -1.0f,  1.0f)); // 4 --+
		mVboPositions.push_back(size * Vec3f( 1.0f, -1.0f,  1.0f)); // 5 +-+
		mVboPositions.push_back(size * Vec3f(-1.0f,  1.0f,  1.0f)); // 6 -++
		mVboPositions.push_back(size * Vec3f( 1.0f,  1.0f,  1.0f)); // 7 +++

		// Define normals
		Vec3f norm0 = Vec3f( 1.0f,  0.0f,  0.0f); // Right
		Vec3f norm1 = Vec3f( 0.0f,  1.0f,  0.0f); // Top
		Vec3f norm2 = Vec3f( 0.0f,  0.0f,  1.0f); // Front
		Vec3f norm3 = Vec3f(-1.0f,  0.0f,  0.0f); // Left
		Vec3f norm4 = Vec3f( 0.0f, -1.0f,  0.0f); // Bottom
		Vec3f norm5 = Vec3f( 0.0f,  0.0f, -1.0f); // Back

		// Set normals
		for (int32_t i = 0; i < 6; i++)
			mVboNormals.push_back(norm0);
		for (int32_t i = 0; i < 6; i++)
			mVboNormals.push_back(norm1);
		for (int32_t i = 0; i < 6; i++)
			mVboNormals.push_back(norm2);
		for (int32_t i = 0; i < 6; i++)
			mVboNormals.push_back(norm3);
		for (int32_t i = 0; i < 6; i++)
			mVboNormals.push_back(norm4);
		for (int32_t i = 0; i < 6; i++)
			mVboNormals.push_back(norm5);

		// Right
		mVboIndices.push_back(1);
		mVboIndices.push_back(3);
		mVboIndices.push_back(5);
		mVboIndices.push_back(5);
		mVboIndices.push_back(3);
		mVboIndices.push_back(7);

		// Top
		mVboIndices.push_back(2);
		mVboIndices.push_back(3);
		mVboIndices.push_back(6);
		mVboIndices.push_back(6);
		mVboIndices.push_back(3);
		mVboIndices.push_back(7);

		// Front
		mVboIndices.push_back(4);
		mVboIndices.push_back(5);
		mVboIndices.push_back(6);
		mVboIndices.push_back(6);
		mVboIndices.push_back(5);
		mVboIndices.push_back(7);

		// Left
		mVboIndices.push_back(0);
		mVboIndices.push_back(2);
		mVboIndices.push_back(4);
		mVboIndices.push_back(4);
		mVboIndices.push_back(2);
		mVboIndices.push_back(6);

		// Bottom
		mVboIndices.push_back(0);
		mVboIndices.push_back(1);
		mVboIndices.push_back(4);
		mVboIndices.push_back(4);
		mVboIndices.push_back(1);
		mVboIndices.push_back(5);

		// Back
		mVboIndices.push_back(0);
		mVboIndices.push_back(1);
		mVboIndices.push_back(2);
		mVboIndices.push_back(2);
		mVboIndices.push_back(1);
		mVboIndices.push_back(3);

		// Set VBO data
		setVboData();
		clearVboData();

	}

	// Cylinder
	RigidCylinder::RigidCylinder(DynamicsWorldRef world, float topRadius, float bottomRadius, float height, int32_t segments, const Vec3f & position, const Quatf & rotation)
		: RigidObject(world, position, rotation)
	{

		// Create body
		mBody = bullet::create(world->getWorld(), topRadius, bottomRadius, height, segments, position, rotation);

		// Set delta size
		float delta = 1.0f / (float)segments;

		// Iterate layers
		for (int32_t p = 0; p < 2; p++) 
		{

			// Choose radius
			float radius = p == 0 ? bottomRadius : topRadius;

			// Iterate segments
			int32_t t = 0;
			for (float theta = delta; t < segments; t++, theta += delta)
			{

				// Set normal
				Vec3f normal(math<float>::cos(2.0f * (float)M_PI * theta), 0.0f, math<float>::sin(2.0f * (float)M_PI * theta));
				mVboNormals.push_back(normal);

				// Set vertex
				Vec3f position(normal.x * radius, (float)p * height, normal.z * radius);
				mVboPositions.push_back(position);

			}

		}

		// Top and bottom center
		mVboPositions.push_back(Vec3f::zero());
		mVboPositions.push_back(Vec3f(0.0f, height, 0.0f));
		int32_t bottomCenter = (int32_t)mVboPositions.size() - 1;
		int32_t topCenter = bottomCenter - 1;

		// Build top face
		for (int32_t t = 0; t < segments; t++) 
		{
			int32_t n = t + 1 >= segments ? 0 : t + 1;
			mVboIndices.push_back(t);
			mVboIndices.push_back(topCenter);
			mVboIndices.push_back(n);
		}

		// Build body
		for (int32_t t = 0; t < segments; t++) 
		{
			int32_t n = t + 1 >= segments ? 0 : t + 1;
			mVboIndices.push_back(t);
			mVboIndices.push_back(segments + t);
			mVboIndices.push_back(n);
			mVboIndices.push_back(n);
			mVboIndices.push_back(segments + t);
			mVboIndices.push_back(segments + n);
		}
			
		// Build bottom face
		for (int32_t t = 0; t < segments; t++) 
		{
			int32_t n = t + 1 >= segments ? 0 : t + 1;
			mVboIndices.push_back(segments + t);
			mVboIndices.push_back(bottomCenter);
			mVboIndices.push_back(segments + n);
		}
		
		// Set VBO data
		setVboData();
		clearVboData();

	}

	// Convex hull mesh
	RigidHull::RigidHull(DynamicsWorldRef world, const TriMesh & mesh, const Vec3f & scale, const Vec3f & position, const Quatf & rotation)
		: RigidObject(world, position, rotation)
	{

		// Change positions to dynamic
		mVboLayout.setDynamicPositions();

		// Define body and VBO
		mBody = bullet::create(world->getWorld(), createConvexHullShape(mesh, scale), position, rotation);
		mVboMesh = gl::VboMesh(mesh, mVboLayout);

		// Scale VBO
		gl::VboMesh::VertexIter vertexIt = mVboMesh.mapVertexBuffer();
		for (uint32_t i = 0; i < mVboMesh.getNumVertices(); i++, ++vertexIt)
			vertexIt.setPosition(vertexIt.getPositionPointer()->xyz() * scale);

	}

	// Concave mesh
	RigidMesh::RigidMesh(DynamicsWorldRef world, const TriMesh & mesh, const Vec3f & scale, float margin, const Vec3f & position, const Quatf & rotation)
		: RigidObject(world, position, rotation)
	{

		// Change positions to dynamic so we can scale the VBO
		mVboLayout.setDynamicPositions();

		// Define body and VBO
		Vec3f halfScale = scale * 0.5f;
		mBody = bullet::create(world->getWorld(), createConcaveMeshShape(mesh, scale, margin), position, rotation);
		mVboMesh = gl::VboMesh(mesh, mVboLayout);

		// Scale VBO
		gl::VboMesh::VertexIter vertexIt = mVboMesh.mapVertexBuffer();
		for (uint32_t i = 0; i < mVboMesh.getNumVertices(); i++, ++vertexIt)
			vertexIt.setPosition(vertexIt.getPositionPointer()->xyz() * scale);

	}

	// Sphere
	RigidSphere::RigidSphere(DynamicsWorldRef world, float radius, int32_t segments, const Vec3f & position, const Quatf & rotation) 
		: RigidObject(world, position, rotation)
	{

		// Create body
		mBody = bullet::create(world->getWorld(), radius, position, rotation);

		// Define steps
		int32_t layers = segments / 2;
		float step = (float)M_PI / (float)layers;
		float delta = ((float)M_PI * 2.0f) / (float)segments;

		// Phi
		int32_t p = 0;
		for (float phi = 0.0f; p <= layers; p++, phi += step)
		{

			// Theta
			int32_t t = 0;
			for (float theta = delta; t < segments; t++, theta += delta)
			{

				// Set vertex
				Vec3f position(
					radius * math<float>::sin(phi) * math<float>::cos(theta),
					radius * math<float>::sin(phi) * math<float>::sin(theta),
					-radius * math<float>::cos(phi));
				mVboPositions.push_back(position);

				// Set normal
				Vec3f normal = position.normalized();
				mVboNormals.push_back(normal);

				// Add indices
				int32_t n = t + 1 >= segments ? 0 : t + 1;
				mVboIndices.push_back(p * segments + t);
				mVboIndices.push_back((p + 1) * segments + t);
				mVboIndices.push_back(p * segments + n);
				mVboIndices.push_back(p * segments + n);
				mVboIndices.push_back((p + 1) * segments + t);
				mVboIndices.push_back((p + 1) * segments + n);

			}

		}

		// Set VBO data
		setVboData();
		clearVboData();

	}

	// Terrain from Surface data
	RigidTerrain::RigidTerrain(DynamicsWorldRef world, const Surface32f & heightField, int32_t stickWidth, int32_t stickLength, float minHeight, float maxHeight, int32_t upAxis, const Vec3f & scale, const Vec3f & position, const Quatf & rotation)
		: RigidObject(world, position, rotation)
	{

		// Create body
		mBody = bullet::create(world->getWorld(), createHeightfieldTerrainShape(heightField, stickWidth, stickLength, 1.0f, minHeight, maxHeight, 1, scale), position, rotation);

		// Get image dimensions
		int32_t height = heightField.getHeight();
		int32_t width = heightField.getWidth();
		float halfHeight = (float)height * 0.5f;
		float halfWidth = (float)width * 0.5f;
		
		// Set grey for dot product
		Vec3f grey(0.3333f, 0.3333f, 0.3333f);

		// Iterate through dimensions
		for (int32_t y = 0; y < height; y++)
			for (int32_t x = 0; x < width; x++) 
			{

				// Get pixel color
				Colorf color(heightField.getPixel(Vec2i(x, y)));
				
				// Add position
				mVboPositions.push_back(Vec3f(
					((float)x - halfWidth) * scale.x, 
					Vec3f(color.r, color.g, color.b).dot(grey) * scale.y, 
					((float)y - halfHeight) * scale.z)
					);

				// Add default normal
				mVboNormals.push_back(Vec3f::zero());

				// Add indices for this quad
				int32_t xn = x + 1 >= width ? 0 : 1;
				int32_t yn = y + 1 >= height ? 0 : 1;
				mVboIndices.push_back(x + height * y);
				mVboIndices.push_back((x + xn) + height * y);
				mVboIndices.push_back((x + xn) + height * (y + yn));
				mVboIndices.push_back(x + height * (y + yn));
				mVboIndices.push_back((x + xn) + height * (y + yn));
				mVboIndices.push_back(x + height * y);

			}

		// Loop through again to set normals
#ifdef CINDER_MSW
		Concurrency::parallel_for(0, height, [=](int32_t y)
		{
#else
		for (int32_t y = 0; y < height; y++)
#endif
			for (int32_t x = 0; x < width; x++)
			{

				// Get vertices of this triangle
				Vec3f vert0 = mVboPositions[mVboIndices[(x + height * y) * 6]];
				Vec3f vert1 = mVboPositions[mVboIndices[((x + 1) + height * y) * 6]];
				Vec3f vert2 = mVboPositions[mVboIndices[((x + 1) + height * (y + 1)) * 6]];

				// Calculate normal
				mVboNormals[x + height * y] = Vec3f((vert1 - vert0).cross(vert1 - vert2).normalized());

			}
#ifdef CINDER_MSW
		});
#else
		}
#endif

		// Set VBO data
		setVboData();
		clearVboData();

	}

	// Torus
	/*RigidTorus::RigidTorus(DynamicsWorldRef world, float innerRadius, float outerRadius, int32_t segments, const Vec3f & position, const Quatf & rotation) 
		: RigidObject(world, position, rotation)
	{

		// Create body
		mBody = create(world->getWorld(), innerRadius, outerRadius, segments, position, rotation);

		// Set VBO data
		//setVboData();
		//clearVboData();

	}*/

	/****** Methods for creating object pointers ******/

	// Box from Vec3f
	RigidObjectRef RigidBox::create(DynamicsWorldRef world, const Vec3f & dimensions, const Vec3f & position, const Quatf & rotation)
	{
		return RigidObjectRef(new RigidBox(world, dimensions, position, rotation));
	}
	CollisionObjectRef createRigidBox(DynamicsWorldRef world, const Vec3f & dimensions, const Vec3f & position, const Quatf & rotation)
	{
		return (CollisionObjectRef)RigidBox::create(world, dimensions, position, rotation);
	}

	// Cylinder
	RigidObjectRef RigidCylinder::create(DynamicsWorldRef world, float topRadius, float bottomRadius, float height, int32_t segments, const Vec3f & position, const Quatf & rotation)
	{
		return RigidObjectRef(new RigidCylinder(world, topRadius, bottomRadius, height, segments, position, rotation));
	}
	CollisionObjectRef createRigidCylinder(DynamicsWorldRef world, float topRadius, float bottomRadius, float height, int32_t segments, const Vec3f & position, const Quatf & rotation)
	{
		return (CollisionObjectRef)RigidCylinder::create(world, topRadius, bottomRadius, height, segments, position, rotation);
	}

	// Hull
	RigidObjectRef RigidHull::create(DynamicsWorldRef world, const TriMesh & mesh, const Vec3f & scale, const Vec3f & position, const Quatf & rotation)
	{
		return RigidObjectRef(new RigidHull(world, mesh, scale, position, rotation));
	}
	CollisionObjectRef createRigidHull(DynamicsWorldRef world, const TriMesh & mesh, const Vec3f & scale, const Vec3f & position, const Quatf & rotation)
	{
		return (CollisionObjectRef)RigidHull::create(world, mesh, scale, position, rotation);
	}

	// Mesh
	RigidObjectRef RigidMesh::create(DynamicsWorldRef world, const TriMesh & mesh, const Vec3f & scale, float margin, const Vec3f & position, const Quatf & rotation)
	{
		return RigidObjectRef(new RigidMesh(world, mesh, scale, margin, position, rotation));
	}
	CollisionObjectRef createRigidMesh(DynamicsWorldRef world, const TriMesh & mesh, const Vec3f & scale, float margin, const Vec3f & position, const Quatf & rotation)
	{
		return (CollisionObjectRef)RigidMesh::create(world, mesh, scale, margin, position, rotation);
	}

	// Sphere
	RigidObjectRef RigidSphere::create(DynamicsWorldRef world, float radius, int32_t segments, const Vec3f & position, const Quatf & rotation)
	{
		return RigidObjectRef(new RigidSphere(world, radius, segments, position, rotation));
	}
	CollisionObjectRef createRigidSphere(DynamicsWorldRef world, float radius, int32_t segments, const Vec3f & position, const Quatf & rotation)
	{
		return (CollisionObjectRef)RigidSphere::create(world, radius, segments, position, rotation);
	}

	// Terrain
	RigidObjectRef RigidTerrain::create(DynamicsWorldRef world, const Surface32f & heightField, int32_t stickWidth, int32_t stickLength, float minHeight, float maxHeight, int32_t upAxis, const Vec3f & scale, const Vec3f & position, const Quatf & rotation)
	{
		return RigidObjectRef(new RigidTerrain(world, heightField, stickWidth, stickLength, minHeight, maxHeight, upAxis, scale, position, rotation));
	}
	CollisionObjectRef createRigidTerrain(DynamicsWorldRef world, const Surface32f & heightField, int32_t stickWidth, int32_t stickLength, float minHeight, float maxHeight, int32_t upAxis, const Vec3f & scale, const Vec3f & position, const Quatf & rotation)
	{
		return (CollisionObjectRef)RigidTerrain::create(world, heightField, stickWidth, stickLength, minHeight, maxHeight, upAxis, scale, position, rotation);
	}

	// Torus
	/*CollisionObjectRef RigidTorus::create(DynamicsWorldRef world, float innerRadius = 5.0f, float outerRadius = 10.0f, int32_t segments = 16, const Vec3f & position, const Quatf & rotation)
	{
		return RigidObjectRef(new RigidTorus(world, innerRadius, outerRadius, segments, position, rotation));
	}*/

}
