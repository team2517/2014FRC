#ifndef SWERVEMODULE_HPP
#define SWERVEMODULE_HPP


struct modulePlug {
	int analogChannel, turnID, moveID;
	
	float offset, xRotation, yRotation;
};


class SwerveModule
{
	float offset;
	float mag, tarTheta, curTheta, diffTheta, turnVel;
	float phi;
	float prevTurnVel;
	bool changeSign;
	float moveTime;
	float xVector;
	float yVector;
	float xBaseVector;
	float yBaseVector;
	float largestMag;
	AnalogChannel *posEncoder; 
	CANJaguar *turnWheel;
	CANJaguar *moveWheel;
	DriverStationLCD *dsLCD;
	Timer *baneTimer;

public:

	SwerveModule(modulePlug);
	
	void setRotation(float x, float y);
	
	float getMagnitude(float leftX, float leftY, float rightX);
	
	void setSpeed(float newMagnitude);
	
	

};
#endif
