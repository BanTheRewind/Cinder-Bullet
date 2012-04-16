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
#include "cinder/app/AppBasic.h"
#include "cinder/gl/Light.h"
#include "cinder/gl/Texture.h"
#include "cinder/Camera.h"
#include "cinder/ImageIo.h"
#include "cinder/ObjLoader.h"
#include "cinder/Rand.h"
#include "cinder/Surface.h"
#include "cinder/TriMesh.h"
#include "cinder/Utilities.h"
#include "CinderBullet.h"
#include "Resources.h"

// This class demonstrates how to use inheritance to expand a body's 
// capabilities. This extends a RigidTerrain to turn it into a dynamic
// landscape.
class DynamicTerrain : public bullet::RigidTerrain
{
public:
	DynamicTerrain( const ci::Channel32f &heightField, float minHeight = -1.0f, float maxHeight = 1.0f, 
		const ci::Vec3f &scale = ci::Vec3f::one(), float mass = 1.0f, const ci::Vec3f &position = ci::Vec3f::zero(), 
		const ci::Quatf &rotation = ci::Quatf() )
		: RigidTerrain( heightField, minHeight, maxHeight, scale, mass, position, rotation )
	{
	}

	// Give access to the terrain channel data and texture coordinates
	ci::Channel32f&				getData() { return mChannel; }
	std::vector<ci::Vec2f>&		getTexCoords() { return mTexCoords; }

	// Bullet automatically updates the terrain from the channel. This 
	// method will update the VBO, as well.
	void						updateVbo()
	{

		// This is the internal method the RigidTerrain uses to
		// generate position and normal data from a channel.
		readChannelData();

		// Buffer new data into VBO
		mVboMesh->bufferPositions( mPositions );
		mVboMesh->bufferNormals( mNormals );
		mVboMesh->bufferTexCoords2d( 0, mTexCoords );

	}
};

// Bullet physics sample application
class BasicSampleApp : public ci::app::AppBasic 
{

public:

	void						draw();
	void						keyDown( ci::app::KeyEvent event );
	void						mouseDown( ci::app::MouseEvent event );
	void						mouseWheel( ci::app::MouseEvent event );
	void						prepareSettings( ci::app::AppBasic::Settings * settings );
	void						resize( ci::app::ResizeEvent event );
	void						setup();
	void						shutdown();
	void						update();
	
private:

	static const uint32_t		GROUND = 5;
	static const uint32_t		MAX_OBJECTS = 300;
	static const uint32_t		MAX_SPHERES = 80;

	ci::CameraPersp				mCamera;
	ci::gl::Light				*mLight;

	void						loadModels();
	ci::TriMesh					mConcave;
	ci::TriMesh					mConvex;

	DynamicTerrain*				mTerrain;

	bullet::CollisionObjectRef	mGround;
	btTransform					mGroundTransform;
	bullet::DynamicsWorldRef	mWorld;

	ci::gl::Texture				mTexSquare;
	ci::gl::Texture				mTexSphere;
	ci::gl::Texture				mTexTerrain;
	
	void						bindTexture( uint32_t index );
	void						unbindTexture( uint32_t index );

};

using namespace bullet;
using namespace ci;
using namespace ci::app;
using namespace std;

void BasicSampleApp::bindTexture( uint32_t index )
{
	if ( GROUND >= 4 ) {
		if ( index == 0 ) {
			mTexTerrain.bind();
		} else {
			mTexSphere.bind();
		}
	} else {
		if ( ( ( GROUND == 0 || GROUND == 3 ) && index > 0 ) || GROUND == 1 ) {
			mTexSquare.bind();
		}
	}
}

void BasicSampleApp::draw()
{
	gl::setViewport( getWindowBounds() );
	gl::setMatrices( mCamera );
	gl::clear( ColorAf::black() );

	gl::pushMatrices();
	gl::rotate( Vec3f( -45.0f, 0.0f, 0.0f ) );
	uint32_t i = 0;
	for ( bullet::Iter object = mWorld->begin(); object != mWorld->end(); ++object, i++ ) {
		gl::pushMatrices();
		glMultMatrixf( object->getTransformMatrix() );
		bindTexture( i );
		gl::draw( object->getVboMesh() );
		unbindTexture( i );
		gl::popMatrices();
	}
	gl::popMatrices();
}

void BasicSampleApp::keyDown( KeyEvent event )
{
	switch ( event.getCode() )
	{
	case KeyEvent::KEY_ESCAPE:
		quit();
		break;
	case KeyEvent::KEY_SPACE:
		writeImage( getAppPath() / "\\frame_" / toString( getElapsedFrames() ) / ".png", copyWindowSurface() );
		break;
	}
}

void BasicSampleApp::loadModels()
{
	ObjLoader loader( loadResource( RES_OBJ_SPHERE )->createStream() );
	loader.load( & mConvex );
	loader = ObjLoader( loadResource( RES_OBJ_TORUS )->createStream() );
	loader.load( & mConcave );
}

// Handles mouse button press
void BasicSampleApp::mouseDown( MouseEvent event )
{

	for ( uint32_t i = 0; i < 10; i++ ) {

		// Set random size and position
		float size = Rand::randFloat( 1.0f, 12.0f );
		Vec3f position = Vec3f( 
			( float )( ( rand() % 200 ) - 100 ), 
			( float )( ( rand() % 50 ) + 200 ), 
			( float )( ( rand() % 200 ) - 100 )
			 );

		// Add a body
		if ( GROUND >= 4 ) {
			bullet::createRigidSphere( mWorld, size, 16, size * size, position );
		} else if ( GROUND == 3 ) {
			bullet::CollisionObjectRef cylinder = bullet::createRigidCylinder( mWorld, Vec3f( size, size * 3, size ), 24, size * size, position );
			btRigidBody* shape = bullet::toBulletRigidBody( cylinder );
			shape->setAngularFactor( 0.95f );
		} else {
			size *= 2.0f;
			bullet::createRigidBox( mWorld, Vec3f::one() * size, size * size, position );
		}
		
	}

}

void BasicSampleApp::mouseWheel( MouseEvent event )
{

	// Zoom
	mCamera.setEyePoint( mCamera.getEyePoint() + Vec3f( 0.0f, 0.0f, event.getWheelIncrement() * 20.0f ) );

}

void BasicSampleApp::prepareSettings( Settings * settings )
{
	settings->setFrameRate( 60.0f );
	settings->setFullScreen( false );
	settings->setResizable( false );
	settings->setWindowSize( 1280, 720 );
}

void BasicSampleApp::resize( ResizeEvent event )
{

	// Reset camera
	mCamera.setPerspective( 60.0f, getWindowAspectRatio(), 1.0f, 5000.0f );
	mCamera.lookAt( Vec3f( 0.0f, 0.0f, -200.0f ), Vec3f::zero() );
	gl::setMatrices( mCamera );

	// Set up OpenGL
	glEnable( GL_BLEND );
	glEnable( GL_DEPTH_TEST );
	gl::enable( GL_TEXTURE_2D );
	glShadeModel( GL_SMOOTH );
	glEnable( GL_POLYGON_SMOOTH );
	glHint( GL_POLYGON_SMOOTH_HINT, GL_NICEST );
	glEnable( GL_LIGHTING );
	glEnable( GL_NORMALIZE );
	gl::enableAlphaBlending();
	glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

}

void BasicSampleApp::setup()
{

	// Set up lighting
	mLight = new gl::Light( gl::Light::DIRECTIONAL, 0 );
	mLight->setDirection( Vec3f( 0.0f, 0.1f, 0.3f ).normalized() );
	mLight->setAmbient( ColorAf( 0.2f, 0.2f, 0.2f, 1.0f ) );
	mLight->setDiffuse( ColorAf( 1.0f, 1.0f, 1.0f, 1.0f ) );
	mLight->enable();

	// Load meshes
	loadModels();

	// Create a Bullet dynamics world
	mWorld = bullet::createWorld();

	// Load texture
	mTexSquare = gl::Texture( loadImage( loadResource( RES_TEX_SQUARE ) ) );
	mTexSphere = gl::Texture( loadImage( loadResource( RES_TEX_SPHERE ) ) );
	mTexTerrain = gl::Texture( loadImage( loadResource( RES_TEX_TERRAIN ) ) );
	mTexTerrain.setWrap( GL_REPEAT, GL_REPEAT );
	mTexTerrain.unbind();

	// Used when generating terrain
	Channel32f heightField;

	// Create and add the wobbly box
	mGroundTransform.setIdentity();
	switch ( GROUND ) {
	case 0:
		mGround = bullet::createRigidBox( mWorld, Vec3f( 200.0f, 35.0f, 200.0f ), 0.0f );
		bullet::createRigidSphere( mWorld, 50.0f, 64, 0.0f, Vec3f( 0.0f, -50.0f, 0.0f ) );
		break;
	case 1:
		mGround = bullet::createRigidHull( mWorld, mConvex, Vec3f::one() * 50.0f, 0.0f );
		break;
	case 2:
		mGround = bullet::createRigidMesh( mWorld, mConcave, Vec3f( 10.0f, 1.0f, 10.0f ), 0.0f, 0.0f );
		break;
	case 3:
		mGround = bullet::createRigidBox( mWorld, Vec3f( 200.0f, 35.0f, 200.0f ), 0.0f );
		break;
	case 4:
		heightField = Channel32f( loadImage( loadResource( RES_IMAGE_HEIGHTFIELD_SM ) ) );
		mGround = bullet::createRigidTerrain( mWorld, heightField, -1.0f, 1.0f, Vec3f( 6.0f, 80.0f, 6.0f ), 0.0f );
		break;
	case 5:

		// ADVANCED: To add a custom class, create a standard pointer to it and 
		// pushBack it into your world. Be sure to delete this pointer when you no loinger need it
		heightField = Channel32f( loadImage( loadResource( RES_IMAGE_HEIGHTFIELD ) ) );
		mTerrain = new DynamicTerrain( heightField, -1.0f, 1.0f, Vec3f( 6.0f, 210.0f, 6.0f ), 0.0f );
		mWorld->pushBack( mTerrain );
		break;

	}

	// Set friction for box
	if ( GROUND < 5 ) {
		btRigidBody* boxBody = bullet::toBulletRigidBody( mGround );
		boxBody->setFriction( 0.95f );
	}

	// Run first resize to initialize view
	resize( ResizeEvent( getWindowSize() ) );

}

void BasicSampleApp::shutdown()
{

	// Clean up
	if ( mLight ) {
		delete mLight;
	}
	if ( mTerrain != 0 ) 
	{
		delete mTerrain;
	}

	// TO DO remove
	exit( 1 );

}

void BasicSampleApp::unbindTexture( uint32_t index )
{
	if ( GROUND == 4 ) {
		if ( index == 0 ) {
			mTexTerrain.unbind();
		} else {
			mTexSphere.unbind();
		}
	} else {
		if ( ( ( GROUND == 0 || GROUND == 3 ) && index > 0 ) || GROUND == 1 ) {
			mTexSquare.unbind();
		}
	}
}

void BasicSampleApp::update()
{

	// Update light
	mLight->update( mCamera );

	if ( GROUND < 3 ) {

		// Set box rotation
		float rotation = math<float>::sin( ( float )getElapsedSeconds() * 0.3333f ) * 0.35f	;
		mGroundTransform.setRotation( btQuaternion( 0.25f, 0.0f, 1.0f + rotation * 0.1f, rotation ) );
		mGroundTransform.setOrigin( btVector3( 0.0f, -60.0f, 0.0f ) );

		// Apply rotation to box
		btRigidBody* body = bullet::toBulletRigidBody( mGround );
		body->getMotionState()->setWorldTransform( mGroundTransform );
		body->setWorldTransform( mGroundTransform );

	} else if ( GROUND == 5 ) {

		// Read data
		Channel32f& input = mTerrain->getData();

		// Get image dimensions
		int32_t height = input.getHeight();
		int32_t width = input.getWidth();
		
		// Create output channel
		Channel32f output = Channel32f( width, height );
		
		// Move pixel value over by one
		for ( int32_t y = 0; y < height; y++ ) {
			for ( int32_t x = 0; x < width; x++ ) {
				float value = input.getValue( Vec2i( x, y ) );
				Vec2i position( ( x + 1 ) % width, ( y + 1 ) % height );
				output.setValue( position, value );
			}

		}

		// Copy new data back to original
		input.copyFrom( output, output.getBounds() );

		vector<Vec2f>& texCoords = mTerrain->getTexCoords();
		Vec2f delta( 1.0f / (float)width, 1.0f / (float)height );
		for ( vector<Vec2f>::iterator uv = texCoords.begin(); uv != texCoords.end(); ++uv ) {
			*uv -= delta;
		}

		// Update terrain VBO
		mTerrain->updateVbo();

	}
	
	// Update dynamics world
	mWorld->update();

	/*Iter iter = mWorld->find( mGround );
	OutputDebugStringA( toString( iter->getPosition().x ).c_str() );
	OutputDebugStringA( "\n" );*/

	// Remove objects
	for ( bullet::Iter object = mWorld->begin(); object != mWorld->end(); ) {
		if ( object != mWorld->begin() && object->getPosition().y < -800.0f ) {
			object = mWorld->erase( object );
		} else {
			++object;
		}
	}

	uint32_t max = GROUND == 4 ? MAX_SPHERES : MAX_OBJECTS;
	if ( mWorld->getNumObjects() > max + 1 ) {
		for ( uint32_t i = 1; i < mWorld->getNumObjects() - MAX_SPHERES; i++ ) {
			mWorld->erase( mWorld->begin() + 1 );
		}
	}

}

CINDER_APP_BASIC( BasicSampleApp, RendererGl )
