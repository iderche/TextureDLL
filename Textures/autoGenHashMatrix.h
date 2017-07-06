/***********************************************************************************************************************************
COPYRIGHT AND PERMISSION NOTICE
Penn Software The Penn Haptic Texture Toolkit
Copyright (C) 2013 The Trustees of the University of Pennsylvania
All rights reserved.

See copyright and permission notice at the end of this file.

Report bugs to Heather Culbertson (hculb@seas.upenn.edu, +1 215-573-6748) or Katherine J. Kuchenbecker (kuchenbe@seas.upenn.edu, +1 215-573-2786)

This code is based on the original TexturePad haptic rendering system designed by Joseph M. Romano.
************************************************************************************************************************************/


/*********************************************************************************
This function reads and parses and specified XML model file. The model information
is stored in a hash table for access by other files.
*********************************************************************************/

#pragma once
#ifndef AUTO_GEN_HASH_MAT_H_
#define AUTO_GEN_HASH_MAT_H_

#include "AccSynthHashMatrix.h"
#include "TappingParameters.h"
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

//#include "sharedMemory.h"
using namespace std;

//------------------------------------------------------------------------FUNCTIONS:

// This function reads data created from surface measurements.  Right now, these
// are all in a local folder, but they will be replaced with a materials database,
// so this will need to be rewritten.
// texArray is a list of material names.  This function assumes that there is a 
// folder named "build" filled with texture information in the executable directory.
// 
AccSynthHashMatrix generateHashMatrix( char( *texArray )[50] )
{

	AccSynthHashMatrix tempmatrix( NUM_TEX );

	float allSpeeds[200]; 
	float allForces[200];

	// Arrays to store lists of Delaunay triangulation vertices
	int DT1[200], DT2[200], DT3[200];

	// Arrays to store AR and MA LSFs
	float modLSF[MAX_COEFF]; 
	float modMALSF[MAX_MACOEFF];

	// More properties to be read in from file
	int numMod;
	int numTri;
	float maxSpeed;
	float maxForce;
	float mu;

	int numCoeff;
	int numMACoeff;
	float variance;
	float gain;
	float modSpeed;
	float modForce;

	string imFilename;

	//Find filepath of executable
	char result[100];
	GetModuleFileName( NULL, result, 100 );
	string::size_type pos1 = string( result ).find_last_of( "\\/" );
	string exeFilepath = string( result ).substr( 0, pos1 );
	string::size_type pos2 = exeFilepath.find_last_of( "\\/" );

	//Find filepath of XML model files
	string baseFilename = exeFilepath.substr( 0, pos2 ) + "\\build\\XML\\Models_";
	string myFilename;

	/********* Fill array with known texture names ***********/
	// divide textures by group

	//Paper
	strcpy_s( texArray[0], 50, "Book" );
	strcpy_s( texArray[1], 50, "Bubble Envelope" );
	strcpy_s( texArray[2], 50, "Cardboard" );
	strcpy_s( texArray[3], 50, "Coffee Filter" );
	strcpy_s( texArray[4], 50, "Dot Paper" );
	strcpy_s( texArray[5], 50, "Folder" );
	strcpy_s( texArray[6], 50, "Gift Box" );
	strcpy_s( texArray[7], 50, "Glitter Paper" );
	strcpy_s( texArray[8], 50, "Greeting Card" );
	strcpy_s( texArray[9], 50, "Masking Tape" );
	strcpy_s( texArray[10], 50, "Paper Bag" );
	strcpy_s( texArray[11], 50, "Paper Plate 1" );
	strcpy_s( texArray[12], 50, "Paper Plate 2" );
	strcpy_s( texArray[13], 50, "Playing Card" );
	strcpy_s( texArray[14], 50, "Resume Paper" );
	strcpy_s( texArray[15], 50, "Sandpaper 100" );
	strcpy_s( texArray[16], 50, "Sandpaper 220" );
	strcpy_s( texArray[17], 50, "Sandpaper 320" );
	strcpy_s( texArray[18], 50, "Sandpaper Aluminum Oxide" );
	strcpy_s( texArray[19], 50, "Textured Paper" );
	strcpy_s( texArray[20], 50, "Tissue Paper" );
	strcpy_s( texArray[21], 50, "Wax Paper" );

	//Plastic
	strcpy_s( texArray[22], 50, "ABS Plastic" );
	strcpy_s( texArray[23], 50, "Binder" );
	strcpy_s( texArray[24], 50, "Candle" );
	strcpy_s( texArray[25], 50, "File Portfolio" );
	strcpy_s( texArray[26], 50, "Frosted Acrylic" );
	strcpy_s( texArray[27], 50, "Nitrile Glove" );
	strcpy_s( texArray[28], 50, "Plastic Mesh 1" );
	strcpy_s( texArray[29], 50, "Plastic Mesh 2" );
	strcpy_s( texArray[30], 50, "Tarp" );
	strcpy_s( texArray[31], 50, "Wavy Acrylic" );

	//Fabric
	strcpy_s( texArray[32], 50, "Athletic Shirt" );
	strcpy_s( texArray[33], 50, "Blanket" );
	strcpy_s( texArray[34], 50, "CD Sleeve" );
	strcpy_s( texArray[35], 50, "Canvas 1" );
	strcpy_s( texArray[36], 50, "Canvas 2" );
	strcpy_s( texArray[37], 50, "Canvas 3" );
	strcpy_s( texArray[38], 50, "Cotton" );
	strcpy_s( texArray[39], 50, "Denim" );
	strcpy_s( texArray[40], 50, "Felt" );
	strcpy_s( texArray[41], 50, "Flannel" );
	strcpy_s( texArray[42], 50, "Fleece" );
	strcpy_s( texArray[43], 50, "Leather 1 Back" );
	strcpy_s( texArray[44], 50, "Leather 1 Front" );
	strcpy_s( texArray[45], 50, "Leather 2 Back" );
	strcpy_s( texArray[46], 50, "Leather 2 Front" );
	strcpy_s( texArray[47], 50, "Microfiber Cloth" );
	strcpy_s( texArray[48], 50, "Nylon Bag" );
	strcpy_s( texArray[49], 50, "Nylon Mesh" );
	strcpy_s( texArray[50], 50, "Pleather" );
	strcpy_s( texArray[51], 50, "Portfolio Cover" );
	strcpy_s( texArray[52], 50, "Silk 1" );
	strcpy_s( texArray[53], 50, "Silk 2" );
	strcpy_s( texArray[54], 50, "Textured Cloth" );
	strcpy_s( texArray[55], 50, "Towel" );
	strcpy_s( texArray[56], 50, "Velcro Hooks" );
	strcpy_s( texArray[57], 50, "Velcro Loops" );
	strcpy_s( texArray[58], 50, "Velvet" );
	strcpy_s( texArray[59], 50, "Vinyl 1" );
	strcpy_s( texArray[60], 50, "Vinyl 2" );
	strcpy_s( texArray[61], 50, "Whiteboard Eraser" );

	//Tile
	strcpy_s( texArray[62], 50, "Floortile 1" );
	strcpy_s( texArray[63], 50, "Floortile 2" );
	strcpy_s( texArray[64], 50, "Floortile 3" );
	strcpy_s( texArray[65], 50, "Floortile 4" );
	strcpy_s( texArray[66], 50, "Floortile 5" );
	strcpy_s( texArray[67], 50, "Floortile 6" );
	strcpy_s( texArray[68], 50, "Floortile 7" );

	//Carpet
	strcpy_s( texArray[69], 50, "Artificial Grass" );
	strcpy_s( texArray[70], 50, "Carpet 1" );
	strcpy_s( texArray[71], 50, "Carpet 2" );
	strcpy_s( texArray[72], 50, "Carpet 3" );
	strcpy_s( texArray[73], 50, "Carpet 4" );

	//Foam
	strcpy_s( texArray[74], 50, "EPDM Foam" );
	strcpy_s( texArray[75], 50, "Pink Foam" );
	strcpy_s( texArray[76], 50, "Polyethylene Foam" );
	strcpy_s( texArray[77], 50, "Scouring Pad" );
	strcpy_s( texArray[78], 50, "Styrofoam" );
	strcpy_s( texArray[79], 50, "Textured Rubber" );

	//Metal
	strcpy_s( texArray[80], 50, "Aluminum Foil" );
	strcpy_s( texArray[81], 50, "Aluminum" );
	strcpy_s( texArray[82], 50, "Metal Mesh" );
	strcpy_s( texArray[83], 50, "Metal Shelving" );
	strcpy_s( texArray[84], 50, "Textured Metal" );
	strcpy_s( texArray[85], 50, "Whiteboard" );

	//Stone
	strcpy_s( texArray[86], 50, "Brick 1" );
	strcpy_s( texArray[87], 50, "Brick 2" );
	strcpy_s( texArray[88], 50, "Ceramic" );
	strcpy_s( texArray[89], 50, "Painted Brick" );
	strcpy_s( texArray[90], 50, "Stone Tile 1" );
	strcpy_s( texArray[91], 50, "Stone Tile 2" );
	strcpy_s( texArray[92], 50, "Terra Cotta" );

	//Carbon Fiber
	strcpy_s( texArray[93], 50, "Carbon Fiber" );
	strcpy_s( texArray[94], 50, "Resin Carbon Fiber" );

	//Wood
	strcpy_s( texArray[95], 50, "Cork" );
	strcpy_s( texArray[96], 50, "MDF" );
	strcpy_s( texArray[97], 50, "Painted Wood" );
	strcpy_s( texArray[98], 50, "Stained Wood" );
	strcpy_s( texArray[99], 50, "Wood" );


	// Loop through all textures
	for( int numSurf = 0; numSurf < NUM_TEX; numSurf++ )
	{
		pugi::xml_document doc;

		myFilename = baseFilename + texArray[numSurf] + ".xml"; 

		pugi::xml_parse_result result = doc.load_file( myFilename.c_str() );

		pugi::xml_node modelSet = doc.child( "modelSet" );

		// friction coefficient
		mu = atof( modelSet.child_value( "mu" ) );
		// number of models
		numMod = atoi( modelSet.child_value( "numMod" ) ); 
		// number of triangles in Delaunay triangulation
		numTri = atoi( modelSet.child_value( "numTri" ) ); 
		// maximum modeled speed
		maxSpeed = atof( modelSet.child_value( "maxSpeed" ) ); 
		// maximum modeled force
		maxForce = atof( modelSet.child_value( "maxForce" ) ); 

		// list of all model speeds
		int count = 0;
		for( pugi::xml_node speedList = modelSet.child( "speedList" ).child( "value" ); 
			 speedList; 
			 speedList = speedList.next_sibling( "value" ) )
		{
			allSpeeds[count] = atof( speedList.child_value() );
			count++;
		}

		// list of all model forces
		count = 0;
		for( pugi::xml_node forceList = modelSet.child( "forceList" ).child( "value" ); forceList; forceList = forceList.next_sibling( "value" ) )
		{
			allForces[count] = atof( forceList.child_value() );
			count++;
		}

		// list of all triangles in Delaunay triangulation
		count = 0;
		for( pugi::xml_node tri = modelSet.child( "tri" ); tri; tri = tri.next_sibling( "tri" ) )
		{
			pugi::xml_node triChild = tri.child( "value" );
			DT1[count] = atoi( triChild.child_value() );
			DT2[count] = atoi( triChild.next_sibling( "value" ).child_value() );
			DT3[count] = atoi( triChild.next_sibling( "value" ).next_sibling( "value" ).child_value() );
			count++;
		}
		// number of AR coefficients
		numCoeff = atoi( modelSet.child_value( "numARCoeff" ) ); 
		// number of MA coefficients
		numMACoeff = atoi( modelSet.child_value( "numMACoeff" ) );
		
		// create a hash table for this surface
		tempmatrix.AddTable( numSurf, numMod, allSpeeds, allForces );

		// for each model in the file
		for( pugi::xml_node model = modelSet.child( "model" ); model; model = model.next_sibling( "model" ) )
		{
			//read all AR LSFs
			count = 0;
			pugi::xml_node ARlsf = model.child( "ARlsf" );
			for( pugi::xml_node ARval = ARlsf.child( "value" ); ARval; ARval = ARval.next_sibling( "value" ) )
			{
				modLSF[count] = atof( ARval.child_value() );
				count++;
			}
			// read all MA LSFs (if needed)
			count = 0;
			pugi::xml_node MAlsf = model.child( "MAlsf" );
			for( pugi::xml_node MAval = MAlsf.child( "value" ); 
				 MAval; 
				 MAval = MAval.next_sibling( "value" ) )
			{
				modMALSF[count] = atof( MAval.child_value() );
				count++;
			}
			gain = atof( model.child( "gain" ).child_value() );			

			variance = atof( model.child( "var" ).child_value() ); //model variance
			modSpeed = atof( model.child( "speedMod" ).child_value() ); //model speed
			modForce = atof( model.child( "forceMod" ).child_value() ); //model force

			//create a hash entry for each model
			AccSynthHashEntry HashEntry( numSurf, modForce, modSpeed, modLSF, modMALSF, variance, gain, numCoeff, numMACoeff, numTri, numMod, DT1, DT2, DT3, maxSpeed, maxForce, mu );
			tempmatrix.AddEntry( HashEntry, numMod, allSpeeds, allForces );
		}
	}
	return tempmatrix;
}

TappingParametersMatrix generateTapMatrix( char( *texArray )[50] )
{
	float * allSpeeds;
	float tapSpeed;
	float aTap[tapL];
	int numTap;

	// Find filepath of executable
	char result[100];
	GetModuleFileName( NULL, result, 100 );
	string::size_type pos1 = string( result ).find_last_of( "\\/" );
	string exeFilepath = string( result ).substr( 0, pos1 );
	string::size_type pos2 = exeFilepath.find_last_of( "\\/" );

	//Find filepath of XML model files
	string baseFilename = exeFilepath.substr( 0, pos2 ) + "\\build\\TappingShift\\Taps_";

	TappingParametersMatrix tapMatrix( NUM_TEX );

	// Loop through all textures
	for( int numSurf = 0; numSurf < NUM_TEX; numSurf++ )
	{
		pugi::xml_document doc;

		// get full filename of tap file
		string myFilename = baseFilename + texArray[numSurf] + ".xml"; 

		pugi::xml_parse_result result = doc.load_file( myFilename.c_str() ); // load tap file

		pugi::xml_node tapSet = doc.child( "tapSet" );

		// number of taps
		numTap = atoi( tapSet.child_value( "numTap" ) ); //number of models
		allSpeeds = new float[numTap];

		// list of all model speeds
		int count = 0;
		for( pugi::xml_node speedList = tapSet.child( "speedList" ).child( "value" ); speedList; speedList = speedList.next_sibling( "value" ) )
		{
			allSpeeds[count] = atof( speedList.child_value() );
			count++;
		}

		// create a hash table for this surface
		tapMatrix.AddTable( numSurf, numTap, allSpeeds );

		for( pugi::xml_node tap = tapSet.child( "tap" ); tap; tap = tap.next_sibling( "tap" ) )
		{
			tapSpeed = atof( tap.child( "tapSpeed" ).child_value() ); //tap speed

			//read all tap accelerations
			count = 0;
			pugi::xml_node tapAccel = tap.child( "tapAccel" );
			for( pugi::xml_node tapval = tapAccel.child( "value" ); tapval; tapval = tapval.next_sibling( "value" ) )
			{
				aTap[count] = atof( tapval.child_value() );
				count++;
			}
			TappingParametersEntry TapEntry( numSurf, numTap, tapSpeed, aTap );
			tapMatrix.AddEntry( TapEntry, numTap, allSpeeds );
		}
	}
	return tapMatrix;
}
#endif

/***********************************************************************************************************************************
COPYRIGHT AND PERMISSION NOTICE
Penn Software The Penn Haptic Texture Toolkit
Copyright (C) 2013 The Trustees of the University of Pennsylvania
All rights reserved.

The Trustees of the University of Pennsylvania (“Penn”) and Heather Culbertson, Juan Jose Lopez Delgado, and Katherine J. Kuchenbecker, the developer (“Developer”) of Penn Software The Penn Haptic Texture Toolkit (“Software”) give recipient (“Recipient”) and Recipient’s Institution (“Institution”) permission to use, copy, and modify the software in source and binary forms, with or without modification for non-profit research purposes only provided that the following conditions are met:

1)	All copies of Software in binary form and/or source code, related documentation and/or other materials provided with the Software must reproduce and retain the above copyright notice, this list of conditions and the following disclaimer.

2)	Recipient shall have the right to create modifications of the Software (“Modifications”) for their internal research and academic purposes only.

3)	All copies of Modifications in binary form and/or source code and related documentation must reproduce and retain the above copyright notice, this list of conditions and the following disclaimer.

4)	Recipient and Institution shall not distribute Software or Modifications to any third parties without the prior written approval of Penn.

5)	Recipient will provide the Developer with feedback on the use of the Software and Modifications, if any, in their research.  The Developers and Penn are permitted to use any information Recipient provides in making changes to the Software. All feedback, bug reports and technical questions shall be sent to:

Heather Culbertson, hculb@seas.upenn.edu, +1 215-573-6748
Katherine J. Kuchenbecker, kuchenbe@seas.upenn.edu, +1 215-573-2786

6)	Recipient acknowledges that the Developers, Penn and its licensees may develop modifications to Software that may be substantially similar to Recipient’s modifications of Software, and that the Developers, Penn and its licensees shall not be constrained in any way by Recipient in Penn’s or its licensees’ use or management of such modifications. Recipient acknowledges the right of the Developers and Penn to prepare and publish modifications to Software that may be substantially similar or functionally equivalent to your modifications and improvements, and if Recipient or Institution obtains patent protection for any modification or improvement to Software, Recipient and Institution agree not to allege or enjoin infringement of their patent by the Developers, Penn or any of Penn’s licensees obtaining modifications or improvements to Software from the Penn or the Developers.

7)	Recipient and Developer will acknowledge in their respective publications the contributions made to each other’s research involving or based on the Software. The current citations for Software are:

Heather Culbertson, Juan Jose Lopez Delgado, and Katherine J. Kuchenbecker. One Hundred Data-Driven Haptic Texture Models and Open-Source Methods for Rendering on 3D Objects. In Proc. IEEE Haptics Symposium, February 2014.

8)	Any party desiring a license to use the Software and/or Modifications for commercial purposes shall contact The Center for Technology Transfer at Penn at 215-898-9591.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS, CONTRIBUTORS, AND THE TRUSTEES OF THE UNIVERSITY OF PENNSYLVANIA "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER, CONTRIBUTORS OR THE TRUSTEES OF THE UNIVERSITY OF PENNSYLVANIA BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

************************************************************************************************************************************/
