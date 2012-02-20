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
#include "BulletUtils.h"

namespace bullet 
{

	// Imports
	using namespace ci;
	using namespace std;

	/****** TRANSFORM ******/

	// Convert Bullet matrix to Cinder matrix for rigid bodies
	Matrix44f getWorldTransform(const btRigidBody * body)
	{
		btTransform trans;
		body->getMotionState()->getWorldTransform(trans);
		Matrix44f matrix;
		trans.getOpenGLMatrix(matrix.m);
		return matrix;
	}

	// Convert Bullet matrix to Cinder matrix for soft bodies
	Matrix44f getWorldTransform(const btSoftBody * body)
	{
		btTransform trans = body->getWorldTransform();
		Matrix44f matrix;
		trans.getOpenGLMatrix(matrix.m);
		return matrix;
	}

	// Convert Cinder vector to Bullet vector
	btVector3 toBulletVector3(const Vec3f & v)
	{
		return btVector3(v.x, v.y, v.z);
	}

	// Convert Bullet vector to Cinder vector
	Vec3f fromBulletVector3(const btVector3 & v)
	{
		return Vec3f(v.x(), v.y(), v.z());
	}

	// Convert Cinder quaternion to Bullet quaternion
	btQuaternion toBulletQuaternion(const Quatf & q)
	{
		return btQuaternion(q.v.x, q.v.y, q.v.z, q.w);
	}

	// Convert Bullet quaternion to Cinder quaternion
	Quatf fromBulletQuaternion(const btQuaternion & q)
	{
		return ci::Quatf(q.getX(), q.getY(), q.getZ(), q.getW());
	}

	// Calculate mass using shape's bounding sphere
	float getMass(const btCollisionShape * shape)
	{

		// Get sphere
		btVector3 center;
		btScalar radius;
		shape->getBoundingSphere(center, radius);
		float radiusf = (float)radius;

		// Find spherical mass
		return radiusf * radiusf * radiusf * (float)M_PI * 4.0f / 3.0f;

	}

	/****** MESH ******/

	// Create terrain from color channel
	btHeightfieldTerrainShape * createHeightfieldTerrainShape(const Surface32f & heightField, int32_t stickWidth, int32_t stickLength, float heightScale, float minHeight, float maxHeight, int32_t upAxis, const Vec3f & scale)
	{

		// Create height field shape from channel data
		btHeightfieldTerrainShape * hfShape = new btHeightfieldTerrainShape(stickWidth, stickLength, (new Channel32f(heightField))->getData(), heightScale, minHeight, maxHeight, upAxis, PHY_FLOAT, false);

		// Scale and return shape
		hfShape->setLocalScaling(bullet::toBulletVector3(scale));
		return hfShape;

	}

	// Creates a concave Bullet mesh from a TriMesh
	btBvhTriangleMeshShape * createConcaveMeshShape(const TriMesh & mesh, const Vec3f & scale, float margin)
	{

		// Create Bullet mesh
		btTriangleMesh * triMesh = new btTriangleMesh(true, false);

		// Add triangles
		std::vector<Vec3f> vertices = mesh.getVertices();
		std::vector<size_t> indices = mesh.getIndices();
		for (uint32_t i = 0; i < mesh.getNumTriangles(); i += 3)
			triMesh->addTriangle(
			bullet::toBulletVector3(vertices[indices[i]]), 
			bullet::toBulletVector3(vertices[indices[i + 1]]), 
			bullet::toBulletVector3(vertices[indices[i + 2]]), 
			true
			);

		// Create mesh shape
		btBvhTriangleMeshShape * shape = new btBvhTriangleMeshShape(triMesh, true, true);

		// Scale and return shape
		shape->setLocalScaling(bullet::toBulletVector3(scale));
		shape->setMargin(margin);
		return shape;

	}

	// Creates a concave Bullet mesh from a list of vertices and indices
	btBvhTriangleMeshShape * createConcaveMeshShape(const std::vector<Vec3f> & vertices, const std::vector<uint32_t> & indices, const Vec3f & scale, float margin)
	{

		// Create Bullet mesh
		btTriangleMesh * triMesh = new btTriangleMesh(true, false);

		// Add triangles
		uint32_t numTriangles = indices.size() / 3;
		for (uint32_t i = 0; i < numTriangles; i += 3)
			triMesh->addTriangle(
			bullet::toBulletVector3(vertices[indices[i]]), 
			bullet::toBulletVector3(vertices[indices[i + 1]]), 
			bullet::toBulletVector3(vertices[indices[i + 2]]), 
			true
			);

		// Create mesh shape
		btBvhTriangleMeshShape * shape = new btBvhTriangleMeshShape(triMesh, true, true);

		// Scale and return shape
		shape->setLocalScaling(bullet::toBulletVector3(scale));
		shape->setMargin(margin);
		return shape;

	}

	// Creates a convex Bullet hull from a TriMesh
	btConvexHullShape * createConvexHullShape(const TriMesh & mesh, const Vec3f & scale)
	{

		// Create hull
		btConvexHullShape * shape = new btConvexHullShape();

		// Add points
		std::vector<Vec3f> vertices = mesh.getVertices();
		for (uint32_t i = 0; i < mesh.getNumVertices(); i++)
			shape->addPoint(bullet::toBulletVector3(vertices[i]));

		// Scale and return shape
		shape->setLocalScaling(bullet::toBulletVector3(scale));
		return shape;

	}

	// Creates a convex Bullet hull from a list of vertices
	btConvexHullShape * createConvexHullShape(const std::vector<Vec3f> & vertices, const Vec3f & scale)
	{

		// Create hull
		btConvexHullShape * shape = new btConvexHullShape();

		// Add points
		for (uint32_t i = 0; i < vertices.size(); i++)
			shape->addPoint(bullet::toBulletVector3(vertices[i]));

		// Scale and return shape
		shape->setLocalScaling(bullet::toBulletVector3(scale));
		return shape;

	}

	/****** RIGID BODIES ******/

	// Creates rigid body from any collision shape
	btRigidBody * create(btDynamicsWorld * world, btCollisionShape * shape, const Vec3f & position, const Quatf & rotation)
	{

		// Calculate inertia
		float mass = getMass(shape);
		btVector3 inertia(0, 0, 0);
		shape->calculateLocalInertia(mass, inertia);

		// Create, add, and return rigid body
		btRigidBody * body = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(mass, new btDefaultMotionState(btTransform(bullet::toBulletQuaternion(rotation), bullet::toBulletVector3(position))), shape, inertia));
		world->addRigidBody(body);
		return body;

	}

	/*! Create rigid body from triangle mesh */
	btRigidBody * create(btDynamicsWorld * world, btBvhTriangleMeshShape * shape, const Vec3f & position, const Quatf & rotation)
	{

		// Calculate inertia
		float mass = getMass(shape);
		btVector3 inertia(0, 0, 0);
		shape->calculateLocalInertia(mass, inertia);

		// Create, add, and return rigid body
		btRigidBody * body = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(mass, new btDefaultMotionState(btTransform(bullet::toBulletQuaternion(Quatf()), bullet::toBulletVector3(position))), shape, inertia));
		world->addRigidBody(body);
		return body;

	}

	// Create rigid body from convex hull shape
	btRigidBody * create(btDynamicsWorld * world, btConvexHullShape * shape, const Vec3f & position, const Quatf & rotation)
	{

		// Calculate inertia
		float mass = getMass(shape);
		btVector3 inertia(0, 0, 0);
		shape->calculateLocalInertia(mass, inertia);

		// Create, add, and return rigid body
		btRigidBody * body = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(mass, new btDefaultMotionState(btTransform(bullet::toBulletQuaternion(Quatf()), bullet::toBulletVector3(position))), shape, inertia));
		world->addRigidBody(body);
		return body;

	}

	// Creates a rigid box
	btRigidBody * create(btDynamicsWorld * world, const Vec3f & size, const Vec3f & position, const Quatf & rotation)
	{

		// Create Bullet box
		btCollisionShape * shape = new btBoxShape(bullet::toBulletVector3(size) / 2.0f);

		// Calculate inertia
		float mass = size.x * size.y * size.z;
		btVector3 inertia(0, 0, 0);
		shape->calculateLocalInertia(mass, inertia);

		// Create, add, and return rigid body
		btRigidBody * body = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(mass, new btDefaultMotionState(btTransform(bullet::toBulletQuaternion(rotation), bullet::toBulletVector3(position))), shape, inertia));
		world->addRigidBody(body);
		return body;

	}

	// Creates rigid cylinder
	btRigidBody * create(btDynamicsWorld * world, float topRadius, float bottomRadius, float height, int32_t segments, const Vec3f & position, const Quatf & rotation)
	{

		// Create cylinder
		btCollisionShape * shape = new btCylinderShape(btVector3(height, topRadius, bottomRadius));

		// Calculate inertia
		float mass = (float)M_PI * math<float>::pow((topRadius + bottomRadius) * 0.5f, 2.0f) * height;
		btVector3 inertia(0, 0, 0);
		shape->calculateLocalInertia(mass, inertia);

		// Create, add, and return rigid body
		btRigidBody * body = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(mass, new btDefaultMotionState(btTransform(bullet::toBulletQuaternion(rotation), bullet::toBulletVector3(position))), shape, inertia));
		world->addRigidBody(body);
		return body;

	}

	// Creates a rigid sphere
	btRigidBody * create(btDynamicsWorld * world, float radius, const Vec3f & position, const Quatf & rotation)
	{

		// Create Bullet sphere
		btCollisionShape * shape = new btSphereShape((btScalar) radius);

		// Configure sphere
		btVector3 inertia(0, 0, 0);
		float mass = radius * radius * radius * (float)M_PI * 4.0f / 3.0f;
		shape->calculateLocalInertia(mass, inertia);

		// Create, add, and return rigid body
		btRigidBody * body = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(mass, new btDefaultMotionState(btTransform(bullet::toBulletQuaternion(rotation), bullet::toBulletVector3(position))), shape, inertia));
		world->addRigidBody(body);
		return body;

	}

	// Creates rigid torus
	/*btRigidBody * create(btDynamicsWorld * world, float innerRadius, float outerRadius, int32_t segments, const Vec3f & position, const Quatf & rotation)
	{

	}*/

	/****** SOFT BODIES ******/

	// Create soft body from arbitrary collision shape
	/*btSoftBody * create(btDynamicsWorld * world, btSoftBodyWorldInfo & info, btCollisionShape * shape, const Vec3f & position, const Quatf & rotation)
	{

		// Create soft body
		btSoftBody * body = new btSoftBody(&info);
		btTransform transform;
		transform.setIdentity();
		transform.setOrigin(bullet::toBulletVector3(position));
		body->transform(transform);

		// Add and return body
		((btSoftRigidDynamicsWorld *)world)->addSoftBody(body);
		return body;

	}

	// Create soft mesh
	btSoftBody * create(btDynamicsWorld * world, btSoftBodyWorldInfo & info, btConvexHullShape * shape, const Vec3f & position, const Quatf & rotation, float mass)
	{

		// Set vertex positions
		btVector3 vertices;
		shape->getVertex(shape->getNumPoints(), vertices);

		// Create soft body
		btSoftBody * body = btSoftBodyHelpers::CreateFromConvexHull(info, & vertices, shape->getNumPoints(), false);

		// Set mass and position
		btTransform transform;
		transform.setIdentity();
		transform.setOrigin(bullet::toBulletVector3(position));
		body->transform(transform);
		body->setTotalMass(mass, true);

		// Add and return soft body
		((btSoftRigidDynamicsWorld *)world)->addSoftBody(body);
		return body;

	}*/

	// Create soft box
	btSoftBody * create(btDynamicsWorld * world, btSoftBodyWorldInfo & info, const Vec3f & dimensions, const Vec3f & position, const Quatf & rotation)
	{

		// Define cube
		const btVector3	hull = bullet::toBulletVector3(dimensions) * 0.5f;
		const btVector3	cube [] = 
		{
			hull * btVector3(-1.0f, -1.0f, -1.0f), 
			hull * btVector3(+1.0f, -1.0f, -1.0f), 
			hull * btVector3(-1.0f, +1.0f, -1.0f), 
			hull * btVector3(+1.0f, +1.0f, -1.0f), 
			hull * btVector3(-1.0f, -1.0f, +1.0f), 
			hull * btVector3(+1.0f, -1.0f, +1.0f), 
			hull * btVector3(-1.0f, +1.0f, +1.0f), 
			hull * btVector3(+1.0f, +1.0f, +1.0f)
		};

		// Create soft cube
		btSoftBody * body = btSoftBodyHelpers::CreateFromConvexHull(info, cube, 8);

		// Set position, rotation, and mass
		btTransform transform;
		transform.setIdentity();
		transform.setRotation(bullet::toBulletQuaternion(rotation));
		transform.setOrigin(bullet::toBulletVector3(position));
		body->transform(transform);
		body->setTotalMass((dimensions.x * 0.5f) * (dimensions.y * 0.5f) * (dimensions.z * 0.5f) * (float)M_PI * 4.0f / 3.0f, true);

		// Add and return soft body
		((btSoftRigidDynamicsWorld *)world)->addSoftBody(body);
		return body;

	}

	// Creates a soft sphere
	btSoftBody * create(btDynamicsWorld * world, btSoftBodyWorldInfo & info, float radius, int32_t segments, const Vec3f & position, const Quatf & rotation)
	{

		// Create soft sphere
		btSoftBody * body = btSoftBodyHelpers::CreateEllipsoid(info, btVector3(0.0f, 0.0f, 0.0f), btVector3(radius, radius, radius), segments);

		// Set position, rotation, and mass
		btTransform transform;
		transform.setIdentity();
		transform.setRotation(bullet::toBulletQuaternion(rotation));
		transform.setOrigin(bullet::toBulletVector3(position));
		body->transform(transform);
		body->setTotalMass(radius * radius * radius * (float)M_PI * 4.0f / 3.0f, true);

		// Add and return soft body
		((btSoftRigidDynamicsWorld *)world)->addSoftBody(body);
		return body;

	}

	// Create soft tetra box
	btSoftBody * create(btDynamicsWorld * world, btSoftBodyWorldInfo & info, const TetraCube & tetraCube, const Vec3f & dimensions, const Vec3f & position, const Quatf & rotation)
	{

		// Define cube
		const btVector3	hull = bullet::toBulletVector3(dimensions) * 0.5f;
		const btVector3	cube [] = 
		{
			hull * btVector3(-1.0f, -1.0f, -1.0f), 
			hull * btVector3(+1.0f, -1.0f, -1.0f), 
			hull * btVector3(-1.0f, +1.0f, -1.0f), 
			hull * btVector3(+1.0f, +1.0f, -1.0f), 
			hull * btVector3(-1.0f, -1.0f, +1.0f), 
			hull * btVector3(+1.0f, -1.0f, +1.0f), 
			hull * btVector3(-1.0f, +1.0f, +1.0f), 
			hull * btVector3(+1.0f, +1.0f, +1.0f)
		};

		// Create soft cube
		btSoftBody * body = btSoftBodyHelpers::CreateFromConvexHull(info, cube, 8);

		// Set position, rotation, and mass
		btTransform transform;
		transform.setIdentity();
		transform.setRotation(bullet::toBulletQuaternion(rotation));
		transform.setOrigin(bullet::toBulletVector3(position));
		body->transform(transform);
		body->setTotalMass((dimensions.x * 0.5f) * (dimensions.y * 0.5f) * (dimensions.z * 0.5f) * (float)M_PI * 4.0f / 3.0f, true);
			
		// Add and return soft body
		((btSoftRigidDynamicsWorld *)world)->addSoftBody(body);
		return body;

	}
	/*
	// Create a soft box from AxisAlignedBox3f
	btSoftBody * create(btDynamicsWorld * world, btSoftBodyWorldInfo & info, const AxisAlignedBox3f & box, const Vec3f & position, const Quatf & rotation)
	{

		// Determine box size
		Vec3f min = box.getMin();
		Vec3f max = box.getMax();
		Vec3f size = Vec3f(math<float>::abs(max.x - min.x), math<float>::abs(max.y - min.y), math<float>::abs(max.z - min.z));

		// Create and return box
		return create(world, info, size, position, rotation);

	}

	// Create a soft sphere from Sphere
	btSoftBody * create(btDynamicsWorld * world, btSoftBodyWorldInfo & info, const Sphere & sphere, int32_t segments, const Vec3f & position, const Quatf & rotation)
	{

		// Create and return soft sphere
		return create(world, info, sphere.getRadius(), segments, position, rotation);		

	}*/

}
