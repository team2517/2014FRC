class SwerveModule
{
	float offset;
	float x, y, mag, phi, tarTheta, curTheta, diffTheta, turnVel;

	float prevTurnVel;
	bool changeSign;
	float moveTime;
	float xVector;
	float yVector;
	float xBaseVector;
	float yBaseVector;
	
	void setRotation(float x, float y)
	{
		xBaseVector = x;
	    yBaseVector = y;
	}
	float getMagnitude(float leftX, float leftY, float rightX)
	{
		xVector = xBaseVector * phi;
		yVector = yBaseVector * phi;
		
	}
	float setSpeed(float newMagnitude)
	{
		
	}
};