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
#include "cinder/Channel.h"
#include "cinder/ImageIo.h"
#include "cinder/ObjLoader.h"
#include "cinder/Rand.h"
#include "cinder/Surface.h"
#include "cinder/TriMesh.h"
#include "cinder/Utilities.h"
#include "CinderBullet.h"
#include "Resources.h"

// Bullet physics sample application
class BasicSampleApp : public ci::app::AppBasic 
{

public:

	// Cinder callbacks
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

	// Lifespan of objects, in seconds
	float						mLifespan;

	// Camera
	ci::CameraPersp				mCamera;

	// Shading
	ci::gl::Light				*mLight;

	// 3D objects
	void						loadModels();
	void						loadTerrain();
	ci::TriMesh					mConcave;
	ci::TriMesh					mConvex;

};

// Imports
using namespace bullet;
using namespace ci;
using namespace ci::app;
using namespace std;

// Render
void BasicSampleApp::draw()
{

	// Set up screen	
	gl::setViewport( getWindowBounds() );
	gl::setMatrices( mCamera );
	gl::clear( ColorAf::black() );

	// Rotate
	gl::pushMatrices();
	gl::rotate( Vec3f( -45.0f, math<float>::sin( ( float )getElapsedSeconds() * 0.3333f ) * 20.0f, 0.0f ) );

	for ( bullet::Iter object = bullet::begin(); object != bullet::end(); ++object ) {
		gl::pushMatrices();
		glMultMatrixf( object->getTransformMatrix() );
		gl::draw( object->getVboMesh() );
		gl::popMatrices();
	}

	// End rotation
	gl::popMatrices();

}

// Handles key press
void BasicSampleApp::keyDown( KeyEvent event )
{

	// Key on key...
	switch ( event.getCode() )
	{
	case KeyEvent::KEY_ESCAPE:
		quit();
		break;
	case KeyEvent::KEY_SPACE:
		writeImage( getAppPath() + "\\frame_" + toString( getElapsedFrames() ) + ".png", copyWindowSurface() );
		break;
	}

}

// Load test models
void BasicSampleApp::loadModels()
{

	// Load OBJ files as meshes
	ObjLoader loader( loadResource( RES_OBJ_SPHERE )->createStream() );
	loader.load( & mConvex );
	loader = ObjLoader( loadResource( RES_OBJ_TORUS )->createStream() );
	loader.load( & mConcave );

}

// Load the ground
void BasicSampleApp::loadTerrain()
{

	// Load heightfield image from resources
	Surface32f heightField = ( Surface32f )Surface( loadImage( loadResource( RES_IMAGE_HEIGHTFIELD ) ) );

	// Set random size and position
	float size = Rand::randFloat( 5.0f, 20.0f );
	Vec3f position = Vec3f( 
		( float )( ( rand() % 400 ) - 200 ), 
		( float )( ( rand() % 100 ) + 150 ), 
		( float )( ( rand() % 400 ) - 200 )
		 );

	// Add terrain
	CollisionObject terrain = bullet::createRigidTerrain( heightField, heightField.getWidth(), heightField.getHeight(), -200, 200, 1, Vec3f( 6.0f, 210.0f, 6.0f ) );
	bullet::toBulletRigidBody( terrain )->setMassProps( 0, btVector3( 0.0f, 0.0f, 0.0f ) );

}

// Handles mouse button press
void BasicSampleApp::mouseDown( MouseEvent event )
{

	// Set random size and position
	float size = Rand::randFloat( 5.0f, 20.0f );
	Vec3f position = Vec3f( 
		( float )( ( rand() % 400 ) - 200 ), 
		( float )( ( rand() % 100 ) + 150 ), 
		( float )( ( rand() % 400 ) - 200 )
		 );

	bullet::createRigidSphere( size, 16, position );
	return;

	// Drop primitive into terrain
	switch ( Rand::randInt( 0, 5 ) )
	{
	case 0:
		bullet::createRigidSphere( size, 16, position );
		break;
	case 1:
		bullet::createRigidBox( Vec3f::one() * size, position );
		break;
	case 2:
		bullet::createRigidHull( mConvex, Vec3f::one() * size * 0.5f, position );
		break;
	case 3:
		bullet::createRigidMesh( mConcave, Vec3f::one() * size * 0.12f, 0.0f, position );
		break;
	case 4:
		bullet::createRigidCylinder( size * 0.5f, size, size * 2.0f, 12, position );
		break;
	}

}

// Handles mouse wheel
void BasicSampleApp::mouseWheel( MouseEvent event )
{

	// Zoom
	mCamera.setEyePoint( mCamera.getEyePoint() + Vec3f( 0.0f, 0.0f, event.getWheelIncrement() * 20.0f ) );

}

// Set up window
void BasicSampleApp::prepareSettings( Settings * settings )
{

	// DO IT!
	settings->setFrameRate( 60.0f );
	settings->setFullScreen( false );
	settings->setWindowSize( 640, 480 );

}

// Handles window resize
void BasicSampleApp::resize( ResizeEvent event )
{

	// Reset camera
	mCamera.setPerspective( 60.0f, getWindowAspectRatio(), 1.0f, 2000.0f );
	mCamera.lookAt( Vec3f( 0.0f, 0.0f, -600.0f ), Vec3f::zero() );
	gl::setMatrices( mCamera );

	// Set up OpenGL
	glEnable( GL_BLEND );
	glEnable( GL_DEPTH_TEST );
	glShadeModel( GL_SMOOTH );
	glEnable( GL_POLYGON_SMOOTH );
	glHint( GL_POLYGON_SMOOTH_HINT, GL_NICEST );
	glEnable( GL_LIGHTING );
	glEnable( GL_NORMALIZE );
	gl::enableAlphaBlending();
	glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

}

// Setup
void BasicSampleApp::setup()
{

	// Set default properties
	mLifespan = 5.0f;

	// Set up OpenGL lighting
	mLight = new gl::Light( gl::Light::DIRECTIONAL, 0 );
	mLight->setDirection( Vec3f( 0.0f, 0.1f, 0.3f ).normalized() );
	mLight->setAmbient( ColorAf( 0.2f, 0.2f, 0.2f, 1.0f ) );
	mLight->setDiffuse( ColorAf( 1.0f, 1.0f, 1.0f, 1.0f ) );
	mLight->enable();

	// Load 3D objects
	loadModels();
	loadTerrain();

	// Run first resize to initialize camera
	resize( ResizeEvent( getWindowSize() ) );

}

// Called on exit
void BasicSampleApp::shutdown()
{

	// Clean up
	if ( mLight ) {
		delete mLight;
	}

}

// Runs update logic
void BasicSampleApp::update()
{

	// Update light
	mLight->update( mCamera );

	// Update dynamics world
	bullet::update();

	for ( bullet::Iter object = bullet::begin(); object != bullet::end(); ) {
		if ( object->getAge() > 5.0 ) {
			object = bullet::erase( object );
		} else {
			++object;
		}
	}

}

// Run application
CINDER_APP_BASIC( BasicSampleApp, RendererGl )
