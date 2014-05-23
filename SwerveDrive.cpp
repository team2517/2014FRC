#include "WPILib.h"
#include "math.h"
#include "controls.h"
#include "offsets.h"
#include "SwerveDrive.hpp"
#define MAXPOWER 					1
#define PI							3.14159
#define FLOFFSET					2.73
#define FROFFSET					3.595
#define BLOFFSET					1.295
#define BROFFSET					3.595


//Functions go here
	
		
		

SwerveDrive::SwerveDrive(modulePlug plugsFL, modulePlug plugsFR,
						 modulePlug plugsBR, modulePlug plugsBL)
{
	plugsFL.offset = FLOFFSET;
	plugsFR.offset = FROFFSET;
	plugsBR.offset = BROFFSET;
	plugsBL.offset = BLOFFSET;
	
	ModuleFL = new SwerveModule(plugsFL);
	ModuleFR = new SwerveModule(plugsFR);
	ModuleBR = new SwerveModule(plugsBR);
	ModuleBL = new SwerveModule(plugsBL);
}

void SwerveDrive::moveDrive(float leftX, float leftY, float rightX)
{
	float mags[4];
	float largestMag;
	int i;
	int j;
	
	mags[0] = ModuleFL->getMagnitude(leftX, leftY, rightX);
	mags[1] = ModuleFR->getMagnitude(leftX, leftY, rightX);
	mags[2] = ModuleBR->getMagnitude(leftX, leftY, rightX);
	mags[3] = ModuleBL->getMagnitude(leftX, leftY, rightX);
	
	for (i = 0; i < 4; i++) 
	{
		if (mags[i] > 1) 
		{
			largestMag = mags[i];
			for (j = 0; j < 4; j++) 
			{
				mags[j] = mags[j] / largestMag;
			}
		}
	}
	
	ModuleFL->setSpeed(mags[0]);
	ModuleFR->setSpeed(mags[1]);
	ModuleBR->setSpeed(mags[2]);
	ModuleBL->setSpeed(mags[3]);
}
