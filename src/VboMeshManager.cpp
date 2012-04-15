// Include header
#include "VboMeshManager.h"

namespace bullet
{
	
	using namespace ci;
	using namespace std;

	// Initialize the texture list
	VboMeshManager::VboMeshList VboMeshManager::sVboMeshList = VboMeshManager::VboMeshList();

	// Find and return a texture
	VboMeshRef VboMeshManager::create( PrimitiveType type, const Vec3f &scale, int32_t segments )
	{

		// Remove unused references
		for ( VboMeshManager::VboMeshList::iterator meshIt = sVboMeshList.begin(); meshIt != sVboMeshList.end(); ) {
			if ( meshIt->second.expired() ) {
				meshIt = sVboMeshList.erase( meshIt );
			} else {
				++meshIt;
			}
		}

		// Find VBO -- this will return an expired weak reference if not found
		PrimitiveInfo info( type, scale, segments );
		VboMeshWeakRef & meshWeakRef = sVboMeshList[ info ];
      
		// Check if the weak texture is expired
		if ( meshWeakRef.expired() ) {

			// Create the VBO
			VboMeshRef meshRef;
			switch ( type ) {
			case PRIMITIVE_BOX:
				meshRef = createBox();
				break;
			case PRIMITIVE_CONE:	
				//meshRef = createCone( scale, segments );
				break;
			case PRIMITIVE_CYLINDER:
				meshRef = createCylinder( segments );
				break;
			case PRIMITIVE_SPHERE:
				meshRef = createSphere( segments );
				break;
			}
        
			// Link the weak pointer to the VBO and return
			meshWeakRef = meshRef; 
			return meshRef;

		}

		// Return the VBO if it exists
		return meshWeakRef.lock();

	}

	VboMeshRef VboMeshManager::create( const vector<uint32_t> &indices, const vector<Vec3f> &positions, 
		const vector<Vec3f> &normals, const vector<Vec2f> &texCoords, GLenum primitiveType )
	{

		ci::gl::VboMesh::Layout layout;
		if ( indices.size() > 0 ) {
			layout.setStaticIndices();
		}
		if ( normals.size() > 0 ) {
			layout.setStaticNormals();
		}
		if ( positions.size() > 0 ) {
			layout.setStaticPositions();
		}
		if ( texCoords.size() > 0 ) {
			layout.setStaticTexCoords2d();
		}

		VboMeshRef mesh( new gl::VboMesh( positions.size(), indices.size(), layout, primitiveType ) );
		if ( indices.size() > 0 ) {
			mesh->bufferIndices( indices );
		}
		if ( normals.size() > 0 ) {
			mesh->bufferNormals( normals );
		}
		if ( positions.size() > 0 ) {
			mesh->bufferPositions( positions );
		}
		if ( texCoords.size() > 0 ) {
			mesh->bufferTexCoords2d( 0, texCoords );
		}

		return mesh;

	}

	VboMeshRef VboMeshManager::createBox()
	{

		// Declare vectors
		vector<uint32_t> indices;
		vector<Vec3f> normals;
		vector<Vec3f> positions;
		vector<Vec2f> texCoords;

		// Define VBO positions
		Vec3f size = Vec3f::one() * 0.5f;
		Vec3f pos0 = Vec3f( 1.0f, 1.0f, 1.0f ) * size;
		Vec3f pos1 = Vec3f( 1.0f, -1.0f, 1.0f ) * size;
		Vec3f pos2 = Vec3f( 1.0f, -1.0f, -1.0f ) * size;
		Vec3f pos3 = Vec3f( 1.0f, 1.0f, -1.0f ) * size;
		Vec3f pos4 = Vec3f( -1.0f, 1.0f, -1.0f ) * size;
		Vec3f pos5 = Vec3f( -1.0f, 1.0f, 1.0f ) * size;
		Vec3f pos6 = Vec3f( -1.0f, -1.0f, -1.0f ) * size;
		Vec3f pos7 = Vec3f( -1.0f, -1.0f, 1.0f ) * size;

		// Define nromals
		Vec3f norm0( 1.0f, 0.0f, 0.0f );
		Vec3f norm1( 0.0f, 1.0f, 0.0f );
		Vec3f norm2( 0.0f, 0.0f, 1.0f );
		Vec3f norm3( -1.0f, 0.0f, 0.0f ); 
		Vec3f norm4( 0.0f, -1.0f, 0.0f ); 
		Vec3f norm5( 0.0f, 0.0f, -1.0f ); 

		// Add positions
		positions.push_back( pos0 ); 
		positions.push_back( pos1 ); 	
		positions.push_back( pos2 ); 	
		positions.push_back( pos3 );

		positions.push_back( pos0 ); 
		positions.push_back( pos3 ); 	
		positions.push_back( pos4 ); 	
		positions.push_back( pos5 );

		positions.push_back( pos0 ); 	
		positions.push_back( pos5 ); 	
		positions.push_back( pos7 ); 	
		positions.push_back( pos1 );

		positions.push_back( pos5 ); 	
		positions.push_back( pos4 ); 	
		positions.push_back( pos6 ); 	
		positions.push_back( pos7 );

		positions.push_back( pos6 ); 	
		positions.push_back( pos2 ); 	
		positions.push_back( pos1 ); 	
		positions.push_back( pos7 );

		positions.push_back( pos2 ); 	
		positions.push_back( pos6 ); 	
		positions.push_back( pos4 ); 	
		positions.push_back( pos3 );

		// Add normals
		for ( uint8_t i = 0; i < 4; i++ ) {
			normals.push_back( norm0 );
		}
		for ( uint8_t i = 0; i < 4; i++ ) {
			normals.push_back( norm1 );
		}
		for ( uint8_t i = 0; i < 4; i++ ) {
			normals.push_back( norm2 );
		}
		for ( uint8_t i = 0; i < 4; i++ ) {
			normals.push_back( norm3 );
		}
		for ( uint8_t i = 0; i < 4; i++ ) {
			normals.push_back( norm4 );
		}
		for ( uint8_t i = 0; i < 4; i++ ) {
			normals.push_back( norm5 );
		}

		// Define texture coordinates
		Vec2f texCoord0( 0.0f, 0.0f );
		Vec2f texCoord1( 1.0f, 0.0f );
		Vec2f texCoord2( 1.0f, 1.0f );
		Vec2f texCoord3( 0.0f, 1.0f );

		// Add texture coordinates
		texCoords.push_back( texCoord3 );
		texCoords.push_back( texCoord2 );
		texCoords.push_back( texCoord1 );
		texCoords.push_back( texCoord0 );

		texCoords.push_back( texCoord2 );
		texCoords.push_back( texCoord1 );
		texCoords.push_back( texCoord0 );
		texCoords.push_back( texCoord3 );

		texCoords.push_back( texCoord3 );
		texCoords.push_back( texCoord2 );
		texCoords.push_back( texCoord1 );
		texCoords.push_back( texCoord0 );

		texCoords.push_back( texCoord2 );
		texCoords.push_back( texCoord1 );
		texCoords.push_back( texCoord0 );
		texCoords.push_back( texCoord3 );

		texCoords.push_back( texCoord1 );
		texCoords.push_back( texCoord0 );
		texCoords.push_back( texCoord3 );
		texCoords.push_back( texCoord2 );
		
		texCoords.push_back( texCoord1 );
		texCoords.push_back( texCoord0 );			
		texCoords.push_back( texCoord3 );
		texCoords.push_back( texCoord2 );

		// Add indices
		indices.push_back( 0 );
		indices.push_back( 1 );
		indices.push_back( 2 );
		indices.push_back( 0 );
		indices.push_back( 2 );
		indices.push_back( 3 );
		
		indices.push_back( 4 );
		indices.push_back( 5 );
		indices.push_back( 6 );
		indices.push_back( 4 );
		indices.push_back( 6 );
		indices.push_back( 7 );
		
		indices.push_back( 8 );
		indices.push_back( 9 );
		indices.push_back( 10 );
		indices.push_back( 8 );
		indices.push_back( 10 );
		indices.push_back( 11 );
		
		indices.push_back( 12 );
		indices.push_back( 13 );
		indices.push_back( 14 );
		indices.push_back( 12 );
		indices.push_back( 14 );
		indices.push_back( 15 );
		
		indices.push_back( 16 );
		indices.push_back( 17 );
		indices.push_back( 18 );
		indices.push_back( 16 );
		indices.push_back( 18 );
		indices.push_back( 19 );
		
		indices.push_back( 20 );
		indices.push_back( 21 );
		indices.push_back( 22 );
		indices.push_back( 20 );
		indices.push_back( 22 );
		indices.push_back( 23 );

		// Set VBO data
		VboMeshRef mesh = VboMeshManager::create( indices, positions, normals, texCoords );

		// Clean up
		indices.clear();
		normals.clear();
		positions.clear();
		texCoords.clear();

		// Return mesh
		return mesh;

	}

	//VboMeshRef VboMeshManager::createCone( const Vec3f &scale )
	//{
	//}

	VboMeshRef VboMeshManager::createCylinder( int32_t segments )
	{

		// Declare vectors
		vector<uint32_t> indices;
		vector<Vec3f> normals;
		vector<Vec3f> positions;
		vector<Vec3f> srcNormals;
		vector<Vec3f> srcPositions;
		vector<Vec2f> srcTexCoords;
		vector<Vec2f> texCoords;

		// Set delta size
		float delta = 1.0f / (float)segments;

		// Iterate layers
		for ( int32_t p = 0; p < 2; p++ ) {

			// Iterate segments
			int32_t t = 0;
			for ( float theta = delta; t < segments; t++, theta += delta ) {

				// Set position
				float t = 2.0f * (float)M_PI * theta;
				Vec3f position( 
					math<float>::cos( t ), 
					(float)p - 0.5f, 
					math<float>::sin( t ) 
					);
				srcPositions.push_back( position );

				// Set normal
				Vec3f normal = position.normalized();
				normal.y = 0.0f;
				srcNormals.push_back( normal );

				// Set tex coord
				Vec2f texCoord( normal.x, position.y + 0.5f );
				srcTexCoords.push_back( texCoord );

			}

		}

		// Top and bottom center
		srcNormals.push_back( Vec3f( 0.0f, -1.0f, 0.0f ) );
		srcNormals.push_back( Vec3f( 0.0f, 1.0f, 0.0f ) );
		srcPositions.push_back( Vec3f( 0.0f, -0.5f, 0.0f ) );
		srcPositions.push_back( Vec3f( 0.0f, 0.5f, 0.0f ) );
		srcTexCoords.push_back( Vec2f( 0.0f, 0.0f ) );
		srcTexCoords.push_back( Vec2f( 0.0f, 1.0f ) );
		int32_t bottomCenter = (int32_t)srcPositions.size() - 1;
		int32_t topCenter = bottomCenter - 1;

		// Build top face
		for ( int32_t t = 0; t < segments; t++ ) {
			int32_t n = t + 1 >= segments ? 0 : t + 1;

			normals.push_back( srcNormals[ topCenter ] );
			normals.push_back( srcNormals[ topCenter ] );
			normals.push_back( srcNormals[ topCenter ] );

			positions.push_back( srcPositions[ t ] );
			positions.push_back( srcPositions[ topCenter ] );
			positions.push_back( srcPositions[ n ] );

			texCoords.push_back( srcTexCoords[ topCenter ] );
			texCoords.push_back( srcTexCoords[ topCenter ] );
			texCoords.push_back( srcTexCoords[ topCenter ] );
		}

		// Build body
		for ( int32_t t = 0; t < segments; t++ ) {
			int32_t n = t + 1 >= segments ? 0 : t + 1;
			
			normals.push_back( srcNormals[ t ] );
			normals.push_back( srcNormals[ segments + t ] );
			normals.push_back( srcNormals[ n ] );
			normals.push_back( srcNormals[ n ] );
			normals.push_back( srcNormals[ segments + t ] );
			normals.push_back( srcNormals[ segments + n ] );

			positions.push_back( srcPositions[ t ] );
			positions.push_back( srcPositions[ segments + t ] );
			positions.push_back( srcPositions[ n ] );
			positions.push_back( srcPositions[ n ] );
			positions.push_back( srcPositions[ segments + t ] );
			positions.push_back( srcPositions[ segments + n ] );

			texCoords.push_back( srcTexCoords[ t ] );
			texCoords.push_back( srcTexCoords[ segments + t ] );
			texCoords.push_back( srcTexCoords[ n ] );
			texCoords.push_back( srcTexCoords[ n ] );
			texCoords.push_back( srcTexCoords[ segments + t ] );
			texCoords.push_back( srcTexCoords[ segments + n ] );
		}
			
		// Build bottom face
		for ( int32_t t = 0; t < segments; t++ ) {
			int32_t n = t + 1 >= segments ? 0 : t + 1;

			normals.push_back( srcNormals[ bottomCenter ] );
			normals.push_back( srcNormals[ bottomCenter ] );
			normals.push_back( srcNormals[ bottomCenter ] );

			positions.push_back( srcPositions[ segments + t ] );
			positions.push_back( srcPositions[ bottomCenter ] );
			positions.push_back( srcPositions[ segments + n ] );

			texCoords.push_back( srcTexCoords[ bottomCenter ] );
			texCoords.push_back( srcTexCoords[ bottomCenter ] );
			texCoords.push_back( srcTexCoords[ bottomCenter ] );
		}
			
		// Set VBO data
		VboMeshRef mesh = VboMeshManager::create( indices, positions, normals, texCoords );
		
		// Clean up
		indices.clear();
		normals.clear();
		positions.clear();
		srcNormals.clear();
		srcPositions.clear();
		srcTexCoords.clear();
		texCoords.clear();

		// Return mesh
		return mesh;

	}

	VboMeshRef VboMeshManager::createSphere( int32_t segments )
	{

		// Declare vectors
		vector<uint32_t> indices;
		vector<Vec3f> normals;
		vector<Vec3f> positions;
		vector<Vec2f> texCoords;

		// Define steps
		int32_t layers = segments / 2;
		float step = (float)M_PI / (float)layers;
		float delta = ((float)M_PI * 2.0f) / (float)segments;

		// Phi
		int32_t p = 0;
		for ( float phi = 0.0f; p <= layers; p++, phi += step ) {

			// Theta
			int32_t t = 0;
			for ( float theta = delta; t < segments; t++, theta += delta )
			{

				// Set position
				Vec3f position(
					math<float>::sin( phi ) * math<float>::cos( theta ),
					math<float>::sin( phi ) * math<float>::sin( theta ),
					-math<float>::cos( phi ) );
				positions.push_back(position);

				// Set normal
				Vec3f normal = position.normalized();
				normals.push_back( normal );

				// Set tex coord
				texCoords.push_back( normal.xy() * 0.5f + Vec2f( 0.5f, 0.5f ) ); 

				// Add indices
				int32_t n = t + 1 >= segments ? 0 : t + 1;
				indices.push_back( p * segments + t );
				indices.push_back( ( p + 1 ) * segments + t );
				indices.push_back( p * segments + n );
				indices.push_back( p * segments + n );
				indices.push_back( ( p + 1 ) * segments + t );
				indices.push_back( ( p + 1 ) * segments + n );

			}

		}

		// Set VBO data
		VboMeshRef mesh = VboMeshManager::create( indices, positions, normals, texCoords );

		// Clean up
		indices.clear();
		normals.clear();
		positions.clear();
		texCoords.clear();

		// Return mesh
		return mesh;

	}

	VboMeshManager::PrimitiveInfo::PrimitiveInfo( VboMeshManager::PrimitiveType type, const ci::Vec3f &scale, int32_t segments )
		: mScale( scale ), mSegments( segments ), mType( type )
	{}

	bool VboMeshManager::PrimitiveInfo::operator==( const VboMeshManager::PrimitiveInfo &rhs ) const
	{
		return ( rhs.mSegments == mSegments && rhs.mType == mType && rhs.mScale.x == mScale.x && 
			rhs.mScale.y == mScale.y && rhs.mScale.z == mScale.z );
	}
	bool VboMeshManager::PrimitiveInfo::operator!=( const VboMeshManager::PrimitiveInfo &rhs ) const
	{
		return !( *this == rhs );
	}
	bool VboMeshManager::PrimitiveInfo::operator<( const VboMeshManager::PrimitiveInfo &rhs ) const
	{
		return *this != rhs;
	}

}
