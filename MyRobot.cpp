#include "WPILib.h"
#include "math.h"
#include "controls.h"
#include "offsets.h"
#include "SwerveModule.cpp"
#include "util.cpp"
#include "util.hpp"
#define FL                      0
#define FR                      1
#define BR                      2
#define BL                      3
#define X                       0
#define Y                       1
#define RAWX					2
#define RAWY					3
#define PI                      3.1415926535
#define PVALUE					1.37
#define IVALUE					0.0
#define DVALUE					0
#define MAXPOWER				1
#define STAGGERDELAY			0.005 //In seconds
#define OFFSETMOVE				.25
#define IDLESPEED				0.0

/* float deadBand(float);		Deadband already defined in util.cpp

struct wheelVector {
	float rawx, x, rawy, y, mag, tarTheta, curTheta, diffTheta, turnVel;

	float prevTurnVel;
	bool changeSign;
	float moveTime;
}; */

class RobotDemo : public SimpleRobot 
{
	Joystick stick; // only joystick
	Joystick manipStick;
	Joystick controlStick;
	CANJaguar turnWheelFL; //Banebots that control the direction of the wheels.
	CANJaguar turnWheelFR;
	CANJaguar turnWheelBR;
	CANJaguar turnWheelBL;
	CANJaguar moveWheelFL; //Cims that control the speed of the wheel.
	CANJaguar moveWheelFR;
	CANJaguar moveWheelBR;
	CANJaguar moveWheelBL;
	CANJaguar shooterMotor;
	Talon pickUpArm1;
	Talon pickUpArm2;
	AnalogChannel posEncFL; //Absolute encoders.  Return angle on scale from 0 to 5 volts.
	AnalogChannel posEncFR;
	AnalogChannel posEncBR;
	AnalogChannel posEncBL;
	Compressor compressor;
	Timer staggerTimer; //Measures the time the code takes to jump from motor to motor.
	Timer baneTimer; //Measures the time the motor has spent at neutral to discharge.
	Timer autoTimer; //Measures the current game time.
	Solenoid shooterPistonA;
	Solenoid shooterPistonB;
	
	//Sets the speed of the respective banebot.
	void turnWheel(int module, float speed)
	{
		if (module == 0)
		{
			turnWheelFL.Set(-speed);
		}
		if (module == 1)
		{
			turnWheelFR.Set(-speed);
		}
		else if (module == 2)
		{
			turnWheelBR.Set(-speed);
		}
		else if (module == 3)
		{
			turnWheelBL.Set(-speed);
		}
	}

	//Sets the speed of the respective cim.
	void moveWheel(int module, float speed)
	{
		if (module == 0)
		{
			moveWheelFL.Set(speed);
		}
		if (module == 1)
		{
			moveWheelFR.Set(-speed);
		}
		else if (module == 2)
		{
			moveWheelBR.Set(speed);
		}
		else if (module == 3)
		{
			moveWheelBL.Set(speed);
		}
	}
	

public:
	RobotDemo() :
		stick(1), manipStick(2),controlStick(3), turnWheelFL(9), turnWheelFR(11), 
		turnWheelBR(45),
		turnWheelBL(30), moveWheelFL(27), moveWheelFR(4),
		moveWheelBR(2), moveWheelBL(12),
		shooterMotor(13), pickUpArm1(1), 
		pickUpArm2(10), posEncFL(3),
		posEncFR(2), posEncBR(4), 
		posEncBL(5), compressor(14, 1), shooterPistonA(1), shooterPistonB(2)
	{
		Watchdog().SetExpiration(1);
		compressor.Start();
	}

/**
 * Drive left & right motors for 2 seconds then stop
 */
	void Autonomous() 
	{
	
		while (IsAutonomous() && IsEnabled()) 
		{
		}
	}

/**
 * Runs the motors with arcade steering. 
 */
	void OperatorControl() 
	{
		Watchdog().SetEnabled(true);
		DriverStationLCD *dsLCD = DriverStationLCD::GetInstance();

		baneTimer.Start();
		staggerTimer.Start();

		float leftStickVec[4];
		float phi;
		float largestMag;
		wheelVector wheel[4];
		int i;
		int j;
		int moduleCounter = 1;
		bool moduleFlag = true;
		bool isButtonPressed;
		bool calibrating = false;
		int calMode = 0;
		float flOff;
		float frOff;
		float brOff;
		float blOff;
		float moveVal = .5;
		isButtonPressed = false;		
		
		for (i = 0; i < 4; i++) 
		{
			wheel[i].changeSign = false;
			wheel[i].prevTurnVel = 0;
		}
		

		while (IsOperatorControl()) 
		{
			Watchdog().Feed();

			if(moduleFlag && !calibrating)
			{
				moduleFlag = false;
				
				//Makes all the left stick vectors have a magnitude of 1, rather than 1.4 in the corners.
				leftStickVec[RAWX] = deadBand(stick.GetRawAxis(1));
				leftStickVec[RAWY] = deadBand(stick.GetRawAxis(2));
				leftStickVec[X] = leftStickVec[RAWX]*sqrt(1-.5*pow(
						leftStickVec[RAWY], 2));
				leftStickVec[Y] = -leftStickVec[RAWY]*sqrt(1-.5*pow(
						leftStickVec[RAWX], 2));
				phi = deadBand(stick.GetRawAxis(3));
				
				if (stick.GetRawButton(5))
				{
					moveVal = .9;
				}
				else
				{
					moveVal = .5;
				}
				
				if (stick.GetRawButton(4))
				{
					leftStickVec[X] = 0;
					leftStickVec[Y] = moveVal;
				} 
				else if (stick.GetRawButton(3))
				{
					leftStickVec[X] = moveVal;
					leftStickVec[Y] = 0;
				}
				else if (stick.GetRawButton(2))
				{
					leftStickVec[X] = 0;
					leftStickVec[Y] = -moveVal;
				}
				else if (stick.GetRawButton(1))
				{
					leftStickVec[X] = -moveVal;
					leftStickVec[Y] = 0;
				}				
				
				//Need to change these values based on center/wheel placement.
				if(stick.GetRawButton(7))
				{
					//Rearranged from front right corner.
					//Front left corner
					wheel[FL].x = .146 * phi;
					wheel[FL].y = .150 * phi;
					wheel[FR].x = .146 * phi;
					wheel[FR].y = .710 * phi;
					wheel[BR].x = .704 * phi;
					wheel[BR].y = .710 * phi;
					wheel[BL].x = .704 * phi;
					wheel[BL].y = .143 * phi;
				}
				else if(stick.GetRawButton(8))
				{
					//Front right corner
					//Actually from measurements.
					wheel[FL].x = .146 * phi;
					wheel[FL].y = -.710 * phi;
					wheel[FR].x = .146 * phi;
					wheel[FR].y = -.150 * phi;
					wheel[BR].x = .704 * phi;
					wheel[BR].y = -.143 * phi;
					wheel[BL].x = .704 * phi;
					wheel[BL].y = -.710 * phi;
				}
				else
				{
					//Center of rotation
					wheel[FL].x = .707 * phi;
					wheel[FL].y = .707 * phi;
					wheel[FR].x = .707 * phi;
					wheel[FR].y = -.707 * phi;
					wheel[BR].x = -.707 * phi;
					wheel[BR].y = -.707 * phi;
					wheel[BL].x = -.707 * phi;
					wheel[BL].y = .707 * phi;
				}
				
				wheel[FL].x += leftStickVec[X];
				wheel[FL].y += leftStickVec[Y];
				wheel[FR].x += leftStickVec[X];
				wheel[FR].y += leftStickVec[Y];
				wheel[BR].x += leftStickVec[X];
				wheel[BR].y += leftStickVec[Y];
				wheel[BL].x += leftStickVec[X];
				wheel[BL].y += leftStickVec[Y];
				
				for (i = 0; i < 4; i++) 
				{
					wheel[i].mag = MAXPOWER * sqrt(pow(wheel[i].x, 2) + pow(wheel[i].y, 2));
				}
	
				for (i = 0; i < 4; i++) 
				{
					if (wheel[i].mag > 1 * MAXPOWER) 
					{
						largestMag = wheel[i].mag;
						for (j = 0; j < 4; j++) 
						{
							wheel[j].mag = MAXPOWER * wheel[j].mag / largestMag;
						}
					}
				}
	
				for (i = 0; i < 4; i++) 
				{
					wheel[i].tarTheta = atan(wheel[i].y / wheel[i].x);
	
					if (wheel[i].x < 0) 
					{
						wheel[i].tarTheta += PI;
					}
				}
	
				wheel[FL].curTheta = -(posEncFL.GetVoltage() - FLOFFSET ) / 5 * 2 * PI;
				wheel[FR].curTheta = -(posEncFR.GetVoltage() - FROFFSET) / 5 * 2 * PI;
				wheel[BR].curTheta = -(posEncBR.GetVoltage() - BROFFSET)/ 5 * 2 * PI;
				wheel[BL].curTheta = -(posEncBL.GetVoltage() - BLOFFSET) / 5 * 2 * PI;
	
				for (i=0; i < 4; i++) 
				{
					wheel[i].diffTheta = wheel[i].tarTheta - wheel[i].curTheta;
	
					if (wheel[i].diffTheta > PI) 
					{
						wheel[i].diffTheta -= 2*PI;
					} 
					else if (wheel[i].diffTheta < -PI) 
					{
						wheel[i].diffTheta += 2*PI;
					}
	
					if (wheel[i].diffTheta > PI/2) 
					{
						wheel[i].diffTheta -= PI;
						wheel[i].mag = wheel[i].mag * -1;
					} 
					else if (wheel[i].diffTheta < -PI/2) 
					{
						wheel[i].diffTheta += PI;
						wheel[i].mag = wheel[i].mag * -1;
					}
	
					wheel[i].turnVel = wheel[i].diffTheta / (PI/2);
					
					if (0 < wheel[i].turnVel && wheel[i].turnVel < .25)
					{
						wheel[i].turnVel = .25;
					} 
					if (0 > wheel[i].turnVel && wheel[i].turnVel > -.25)
					{
						wheel[i].turnVel = -.25;
					}
					if (fabs(wheel[i].diffTheta) < PI/45 )
					{
						wheel[i].turnVel = 0;
					}
					if (((wheel[i].turnVel > 0 && wheel[i].prevTurnVel < 0)
							|| (wheel[i].turnVel < 0&& wheel[i].prevTurnVel> 0)) 
							&& !wheel[i].changeSign)
					{
						wheel[i].changeSign = true;
						wheel[i].moveTime = baneTimer.Get() + .1;
					}
					if (wheel[i].changeSign) 
					{
						wheel[i].turnVel = 0;
						if (wheel[i].moveTime < baneTimer.Get()) 
						{
							wheel[i].changeSign = false;
						}
					}
				}
				
				if (calibrating != true)
				{
					dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Mag: %.1f, %.1f, %.1f, %.1f        ",
						wheel[FL].mag, wheel[FR].mag, wheel[BL].mag, wheel[BR].mag);
					dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Diff: %f        ",
						wheel[FL].diffTheta);
					dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "Coords: (%3.2f,%3.2f)        ",
						wheel[FL].x, wheel[FL].y);
					dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "Turn Val: %f         ", wheel[FL].turnVel);
					dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "EncoderPos: %f        ",
						moveWheelFL.GetPosition());
					dsLCD->UpdateLCD();
				}
				
				
				for(i=0; i<4; i++)
				{
					if (!(wheel[i].x == 0 && wheel[i].y == 0))
					{
						turnWheel(i, wheel[i].turnVel);
						moveWheel(i, wheel[i].mag);
					}
					else
					{
						turnWheel(i, 0);
						moveWheel(i, 0);
					}
				}
		        
		        for(i=0; i<4; i++)
	    		{
	    			wheel[i].prevTurnVel = wheel[i].turnVel;
	    		}
			}
			if (staggerTimer.Get() > STAGGERDELAY)
			{
				moduleCounter++;
				moduleFlag = true;
				staggerTimer.Reset();
				
				if(moduleCounter > 4)
				{
					moduleCounter = 1;
				}
			}
		
			if (stick.GetRawButton(6) && !calibrating)
			{
				pickUpArm1.Set(.8);
				pickUpArm2.Set(.8);
			}
			else if (stick.GetRawButton(5) && !calibrating)
			{
				pickUpArm1.Set(-.8);
				pickUpArm2.Set(-.8);
			}
			else
			{
				pickUpArm1.Set(0);
				pickUpArm2.Set(0);
			}
	
			if (manipStick.GetRawButton(3) && !calibrating) 
			{
				shooterMotor.Set(-.25);
			}
			else if(manipStick.GetRawButton(2) && !calibrating)
			{
				shooterMotor.Set(.25);
			}
			else
			{
				shooterMotor.Set(manipStick.GetRawAxis(2));
			}
			
			
			if (manipStick.GetRawButton(1) && !calibrating) 
			{
				shooterPistonA.Set(true);
				shooterPistonB.Set(false);
			}
			else
			{
				shooterPistonA.Set(false);
				shooterPistonB.Set(true);
			}
			
		}
	}
/**
 * Runs during test mode
 */
	void Test() 
	{
	
	}
};

	START_ROBOT_CLASS(RobotDemo)
	;

float deadBand(float axisValue) 
{
	if (axisValue < -.05 || axisValue> .05)
	{
		return axisValue;
	}
	else
	{
		return 0.0;
	}
}


