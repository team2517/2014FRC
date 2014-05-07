class SwerveModule
{
	float offset;
	float x, y, mag, tarTheta, curTheta, diffTheta, turnVel;
	float phi;
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
		phi = rightX;
		xVector = xBaseVector * phi;
		yVector = yBaseVector * phi;
		x += leftX;
		y += leftY;
		mag = MAXPOWER * sqrt(pow(x, 2) + pow(y, 2));
		return mag;		
	}
	float setSpeed(float newMagnitude)
	{
		x = 
	}
};