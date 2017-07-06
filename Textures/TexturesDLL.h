
#include "TexturesManager.h"

#define EXPORT __declspec(dllexport)
//------------------------------------------------------------------------FUNCTIONS:

extern "C"
{
	EXPORT void __stdcall blarg( char* buffer );
	
	EXPORT void __stdcall calculateTap( float tapSpeed, float* buffer );

	EXPORT void __stdcall init();
	
	EXPORT void __stdcall setTextureNumber( int textureNumber );

	EXPORT void __stdcall updateMovement( float tangentSpeed, float normalForce );

	EXPORT double __stdcall vibrations();
}