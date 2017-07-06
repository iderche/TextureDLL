
#pragma once

#ifndef TEXT_MANAGER
#define TEXT_MANAGER

#include <iostream>
#include <tuple>
#include <vector>

#include "AccSynthHashMatrix.h"
#include "TappingParameters.h"

#include "boost\random.hpp"
#include "boost\random\normal_distribution.hpp"

class TexturesManager
{
//-------------------------------------------------------------------CONSTANTS:

private:
	// Duration of transient force (in ms)
	const int TAP_LENGTH = 100;

//----------------------------------------------------------------------FIELDS:

private:

	boost::mt19937 rng;

	bool isInitialized;

	char texArray[NUM_TEX][50];
	float filtCoeff[MAX_COEFF];
	float filtMACoeff[MAX_MACOEFF];
	AccSynthHashMatrix myMatrix;
	TappingParametersMatrix myTapMatrix;

	int textureNumber;
	float* taps;
	float filtVariance;
	float filtGain;
	int coeffNum;
	int MAcoeffNum;

	//TODO: these should be queues
	std::vector <float> outputHist;
	std::vector <float> excitationHist;

//----------------------------------------------------CONSTRUCTORS/DESTRUCTORS:

public:
	TexturesManager();
	~TexturesManager();
	
//---------------------------------------------------------------------METHODS:

public:
	void calculateTap( float tapSpeed, float* buffer );
	void init();
	void setTextureNumber( int textureNumber );
	void updateMovement( float tangentSpeed, float normalForce );
	double vibrations();
private:

};

#endif