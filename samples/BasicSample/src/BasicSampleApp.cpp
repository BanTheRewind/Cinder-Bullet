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

	static const uint32_t		GROUND = 3;
	static const uint32_t		MAX_SPHERES = 80;

	ci::CameraPersp				mCamera;
	ci::gl::Light				*mLight;

	void						loadModels();
	ci::TriMesh					mConcave;
	ci::TriMesh					mConvex;

	bullet::CollisionObjectRef	mBox;
	btTransform					mBoxTransform;
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
	if ( GROUND == 3 ) {
		if ( index == 0 ) {
			mTexTerrain.bind();
		} else {
			mTexSphere.bind();
		}
	} else {
		if ( ( GROUND == 0 && index > 1 ) || GROUND == 1 ) {
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
		writeImage( getAppPath() + "\\frame_" + toString( getElapsedFrames() ) + ".png", copyWindowSurface() );
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
		if ( GROUND == 3 ) {
			bullet::createRigidSphere( mWorld, size, 16, size * size, position );
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
	mTexTerrain.unbind();

	// Create and add the wobbly box
	mBoxTransform.setIdentity();
	switch ( GROUND ) {
	case 0:
		mBox = bullet::createRigidBox( mWorld, Vec3f( 200.0f, 35.0f, 200.0f ), 0.0f );
		bullet::createRigidSphere( mWorld, 50.0f, 64, 0.0f, Vec3f( 0.0f, -50.0f, 0.0f ) );
		break;
	case 1:
		mBox = bullet::createRigidHull( mWorld, mConvex, Vec3f::one() * 50.0f, 0.0f );
		break;
	case 2:
		mBox = bullet::createRigidMesh( mWorld, mConcave, Vec3f( 10.0f, 1.0f, 10.0f ), 0.0f, 0.0f );
		break;
	case 3:
		Channel32f heightField( loadImage( loadResource( RES_IMAGE_HEIGHTFIELD_SM ) ) );
		mBox = bullet::createRigidTerrain( mWorld, heightField, -1.0f, 1.0f, Vec3f( 6.0f, 80.0f, 6.0f ), 0.0f );
		break;
	}

	// Set friction for box
	btRigidBody* boxBody = bullet::toBulletRigidBody( mBox );
	boxBody->setFriction( 0.95f );

	// Run first resize to initialize view
	resize( ResizeEvent( getWindowSize() ) );

}

void BasicSampleApp::shutdown()
{

	// Clean up
	if ( mLight ) {
		delete mLight;
	}

	// TO DO remove
	exit( 1 );

}

void BasicSampleApp::unbindTexture( uint32_t index )
{
	if ( GROUND == 3 ) {
		if ( index == 0 ) {
			mTexTerrain.unbind();
		} else {
			mTexSphere.unbind();
		}
	} else if ( GROUND == 0 ) {
		if ( ( GROUND == 0 && index > 1 ) || GROUND == 1 ) {
			mTexSquare.unbind();
		}
	}
}

void BasicSampleApp::update()
{

	// Update light
	mLight->update( mCamera );

	// If ground is not terrain..
	if ( GROUND < 3 ) {

		// Set box rotation
		float rotation = math<float>::sin( ( float )getElapsedSeconds() * 0.3333f ) * 0.35f	;
		mBoxTransform.setRotation( btQuaternion( 0.25f, 0.0f, 1.0f + rotation * 0.1f, rotation ) );
		mBoxTransform.setOrigin( btVector3( 0.0f, -60.0f, 0.0f ) );

		// Apply rotation to box
		btRigidBody* body = bullet::toBulletRigidBody( mBox );
		body->getMotionState()->setWorldTransform( mBoxTransform );
		body->setWorldTransform( mBoxTransform );

	}
	
	// Update dynamics world
	mWorld->update();

	/*Iter iter = mWorld->find( mBox );
	OutputDebugStringA( toString( iter->getPosition().x ).c_str() );
	OutputDebugStringA( "\n" );*/

	// Remove objects
	if ( GROUND == 3 ) {
		if ( mWorld->getNumObjects() > MAX_SPHERES + 1 ) {
			for ( int32_t i = 1; i < mWorld->getNumObjects() - MAX_SPHERES; i++ ) {
				mWorld->erase( mWorld->begin() + 1 );
			}
		}
	} else {
		for ( bullet::Iter object = mWorld->begin(); object != mWorld->end(); ) {
			if ( object != mWorld->begin() && object->getPosition().y < -800.0f ) {
				object = mWorld->erase( object );
			} else {
				++object;
			}
		}
	}

}

CINDER_APP_BASIC( BasicSampleApp, RendererGl )
