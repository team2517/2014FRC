#include "WPILib.h"
#include "math.h"
#include "controls.h"
#include "offsets.h"
#include "SwerveModule.hpp"
#define MAXPOWER 					1
#define PI							3.14159
#define FLOFFSET					2.73
#define FROFFSET					3.595
#define BLOFFSET					1.295
#define BROFFSET					3.595


void SwerveModule::setRotation(float x, float y)
{
	xBaseVector = x;
    yBaseVector = y;
}

float SwerveModule::getMagnitude(float leftX, float leftY, float rightX)
{
	
	phi = rightX;
	xVector = xBaseVector * phi;
	yVector = yBaseVector * phi;
	xVector += leftX;
	yVector += leftY;
	mag = MAXPOWER * sqrt(pow(xVector, 2) + pow(yVector, 2));
	return mag;		
}


/*
 *Arguments: 
 *   1: Positon encoder analog
 *   2:  
 * 
 * */

void SwerveModule::setSpeed(float newMagnitude)
{
	tarTheta = atan2(yVector, xVector);
	curTheta = -(posEncoder->GetVoltage() - offset ) / 5 * 2 * PI;

	//	Code Snippet
	diffTheta = tarTheta - curTheta;
		
	if (diffTheta > PI) 
	{
		diffTheta -= 2*PI;
	} 
	else if (diffTheta < -PI) 
	{
		diffTheta += 2*PI;
	}

	if (diffTheta > PI/2) 
	{
		diffTheta -= PI;
		mag = mag * -1;
	} 
	else if (diffTheta < -PI/2) 
	{
		diffTheta += PI;
		mag = mag * -1;
	}

	turnVel = diffTheta / (PI/2);
	
	if (0 < turnVel && turnVel < .25)
	{
		turnVel = .25;
	} 
	if (0 > turnVel && turnVel > -.25)
	{
		turnVel = -.25;
	}
	if (fabs(diffTheta) < PI/45 )
	{
		turnVel = 0;
	}
	if (((turnVel > 0 && prevTurnVel < 0)
			|| (turnVel < 0&& prevTurnVel> 0)) 
			&& !changeSign)
	{
		changeSign = true;
		moveTime = baneTimer->Get() + .1; 				
	}
	if (changeSign) 
	{
		turnVel = 0;
			if (moveTime < baneTimer->Get()) 
		{
			changeSign = false;
		}
	}
	
	//	/Code Snippet
	
	
	if (!(xVector == 0 && yVector == 0))
	{
		turnWheel->Set(turnVel);							
		moveWheel->Set(mag);
	}
	else
	{
		turnWheel->Set(0);
		moveWheel->Set(0);
	}
	
	prevTurnVel = turnVel;
	
	
		
		
}

SwerveModule::SwerveModule(modulePlug connections)
{
	posEncoder = new AnalogChannel(connections.analogChannel);
	offset = connections.offset;
	turnWheel = new CANJaguar(connections.turnID);
	moveWheel = new CANJaguar(connections.moveID);
	baneTimer = new Timer();				
	baneTimer->Start();
	xBaseVector = connections.xRotation;
	yBaseVector = connections.yRotation;
}
