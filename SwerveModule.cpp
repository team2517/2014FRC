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
float SwerveModule::setSpeed(float newMagnitude)
{
	tarTheta = atan2(yVector, xVector);
	curTheta = -(posEncoder->GetVoltage() - FLOFFSET ) / 5 * 2 * PI;
	
	
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
//			moveTime = baneTimer.Get() + .1; 				**FIX BANETIMER
	}
	if (changeSign) 
	{
		turnVel = 0;
//			if (moveTime < baneTimer.Get()) 
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
		
		
}

SwerveModule::SwerveModule(int anaChan, int turnJagID, int moveJagID)
{
	posEncoder = new AnalogChannel(anaChan);
	turnWheel = new CANJaguar(turnJagID);
	moveWheel = new CANJaguar(moveJagID);
//	baneTimer = new Timer();					FIRST PROP
//	baneTimer->Start();
}
