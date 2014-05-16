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
	Timer *baneTimer;

public:

	SwerveModule(int, int, int);
	
	void setRotation(float x, float y);
	
	float getMagnitude(float leftX, float leftY, float rightX);
	
	float setSpeed(float newMagnitude);
	
	

};
