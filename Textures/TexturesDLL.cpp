#include "TexturesDLL.h"
#include <string>
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include "shared.h"
#include "pugixml.hpp"
#include <string.h>
#include <Windows.h>
#include <vector>
#include <errno.h>
TexturesManager texturesManager;

void __stdcall blarg( char* buffer )
{
	char result[100];
	GetModuleFileName( NULL, result, 100 );
	std::string::size_type pos1 = std::string( result ).find_last_of( "\\/" );
	std::string exeFilepath = std::string( result ).substr( 0, pos1 );
	std::string::size_type pos2 = exeFilepath.find_last_of( "\\/" );
	std::string baseFilename = exeFilepath.substr( 0, pos2 ) + "\\build\\TappingShift\\Taps_";

	strcpy_s( buffer, 500, baseFilename.c_str() );
	//strcpy_s( buffer, 500, "hi dude" );
}

void __stdcall calculateTap( float tapSpeed, float* buffer )
{
	texturesManager.calculateTap( tapSpeed, buffer );
}

void __stdcall init()
{
	texturesManager.init();
}

void __stdcall setTextureNumber( int textureNumber )
{
	texturesManager.setTextureNumber( textureNumber );
}

void __stdcall updateMovement( float tangentSpeed, float normalForce )
{
	texturesManager.updateMovement( tangentSpeed, normalForce );
}

double __stdcall vibrations()
{
	return texturesManager.vibrations();
}