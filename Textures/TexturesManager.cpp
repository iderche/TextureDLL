
#include "TexturesManager.h"

// TODO fix this class so I can include it in my heade without LNK errors
#include "autoGenHashMatrix.h"


//------------------------------------------------------------------------CONSTANTS:

//---------------------------------------------------------------------------FIELDS:

//---------------------------------------------------------CONSTRUCTORS/DESTRUCTORS:

TexturesManager::TexturesManager()
{
	isInitialized = false;
}

TexturesManager::~TexturesManager()
{
}

//--------------------------------------------------------------------------METHODS:

void TexturesManager::calculateTap( float tapSpeed, float* buffer )
{
	// interpolates entire 100 ms long tap
	float* localBuffer = myTapMatrix.HashAndInterp2( textureNumber, tapSpeed );
	
	for( int i = 0; i < TAP_LENGTH; i++ )
	{
		buffer[i] = localBuffer[i];
	}
}

void TexturesManager::init()
{
	//TODO: This initialization will change. Instead of parsing a local xml file, we
	//		will be reading from a database.  This may happen only once during init,
	//		or we may want to load in new textures regularly and regenerate these
	//		matricies.  These two functions are in temp file "autoGenHashMatrix.h"
	myMatrix = generateHashMatrix( texArray );
	myTapMatrix = generateTapMatrix( texArray );

	isInitialized = true;
}

void TexturesManager::setTextureNumber( int newTextureNumber )
{
	textureNumber = newTextureNumber;
}

void TexturesManager::updateMovement( float tangentSpeed, float normalForce )
{
	// call when force and speed are updated, doesn't have to be called each loop; function definition in "AccSynthHashMatrix.cpp"
	myMatrix.HashAndInterp2( textureNumber, 
							 tangentSpeed, 
							 normalForce, 
							 filtCoeff, 
							 filtMACoeff );

	tie( coeffNum, 
		 MAcoeffNum, 
		 filtVariance, 
		 filtGain ) = myMatrix.HashAndInterp2( textureNumber, 
											   tangentSpeed, 
											   normalForce, 
											   filtCoeff, 
											   filtMACoeff ); 

}

double TexturesManager::vibrations()
{
	double output = 0.0;
	double excitation = 0.0;
	double rgen_mean = 0.;
	boost::mt19937 generator;

	//generate Gaussian random number with power equal to interpolation model variance
	boost::normal_distribution<> nd( rgen_mean, sqrt( filtVariance ) );
	boost::variate_generator<boost::mt19937&, 
							 boost::normal_distribution<>> var_nor( rng, nd );
	excitation = var_nor();
	output = 0.0;

	//if the size of output history is less than the number of AR coefficients, append zeros
	if( outputHist.size()<(unsigned int)MAX_COEFF )
	{
		int subt = MAX_COEFF - outputHist.size();
		for( int j = 0; j < subt; j++ )
		{
			outputHist.push_back( 0.0 );
		}
	}
	// if the size of excitation history is less than the number of MA coefficients, 
	// append zeros
	if( excitationHist.size() < (unsigned int)MAX_MACOEFF ) 
	{
		int subt = MAX_MACOEFF - excitationHist.size();
		for( int j = 0; j < subt; j++ ) 
		{
			excitationHist.push_back( 0.0 );
		}
	}
	//apply AR coefficients to history of output values
	for( int i = 0; i < coeffNum; i++ )
	{
		output += outputHist.at( i ) * ( -filtCoeff[i] );
	}
	//if applicable, also apply MA coefficients to history of excitation values
	output += excitation*filtGain;
	for( int i = 0; i < MAcoeffNum; i++ )
	{
		output += excitationHist.at( i ) * ( filtMACoeff[i] )*filtGain;
	}
	
	// if the size of output history is greater than the number of AR coefficients, 
	// make the extra values zero so we're not storing junk
	if( outputHist.size() > (unsigned int)coeffNum )
	{
		for( unsigned int kk = coeffNum; kk < outputHist.size(); kk++ )
			outputHist.at( kk ) = 0.0;
	}
	// if the size of excitation history is greater than the number of MA coefficients,
	// make the extra values zero so we're not storing junk
	if( excitationHist.size() > (unsigned int)MAcoeffNum )
	{
		for( unsigned int kk = MAcoeffNum; kk < excitationHist.size(); kk++ )
			excitationHist.at( kk ) = 0.0;
	}

	//TODO: Replace these vectors with queues (avoid linear time insert)

	// remove the last element of our output vector
	outputHist.pop_back();
	excitationHist.pop_back();
	// push our new ouput value onto the front of our vector stack
	outputHist.insert( outputHist.begin(), output );
	excitationHist.insert( excitationHist.begin(), excitation );

	return output; //this is the output vibration value (in m/s^2)
}
