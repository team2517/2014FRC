#include "WPILib.h"
#include "math.h"
#include "controls.h"
#include "offsets.h"
//#include "util.h"
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

float deadBand(float);

struct wheelVector {
	float rawx, x, rawy, y, mag, tarTheta, curTheta, diffTheta, turnVel;

	float prevTurnVel;
	bool changeSign;
	float moveTime;
};

/**
 * This is a demo program showing the use of the RobotBase class.
 * The SimpleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 */
class RobotDemo : public SimpleRobot {
	Joystick stick; // only joystick
	Joystick manipStick;
	Joystick controlStick;
	CANJaguar turnWheelFL;
	CANJaguar turnWheelFR;
	CANJaguar turnWheelBR;
	CANJaguar turnWheelBL;
	CANJaguar moveWheelFL;
	CANJaguar moveWheelFR;
	CANJaguar moveWheelBR;
	CANJaguar moveWheelBL;
	CANJaguar shooterMotor1;
	CANJaguar shooterMotor2;
	Talon pickUpArm1;
	Talon pickUpArm2;
	AnalogChannel posEncFL;
	AnalogChannel posEncFR;
	AnalogChannel posEncBR;
	AnalogChannel posEncBL;
	Timer staggerTimer;
	Timer baneTimer;
	Timer autoTimer;
	//float magmodifier;

public:
	RobotDemo() :
		stick(1), manipStick(2),controlStick(3), turnWheelFL(9), turnWheelFR(11), 
		turnWheelBR(45),
			turnWheelBL(30), moveWheelFL(27), moveWheelFR(4),
			moveWheelBR(2), moveWheelBL(12), 
			shooterMotor1(13), shooterMotor2(46),
			pickUpArm1(1), pickUpArm2(10),
			posEncFL(3), posEncFR(2),
			posEncBR(4), posEncBL(5) {
		Watchdog().SetExpiration(1);
	}

	/**
	 * Drive left & right motors for 2 seconds then stop
	 */
	void Autonomous() {
		
		Watchdog().SetEnabled(true);
		DriverStationLCD *dsLCD = DriverStationLCD::GetInstance();

		baneTimer.Start();
		staggerTimer.Start();
		autoTimer.Start();

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
		
		while (IsAutonomous() && IsEnabled()) {
			Watchdog().Feed();
			
			if(moduleFlag && !calibrating)
			{
				dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Time %.2f        ",
									autoTimer.Get());
			
				moduleFlag = false;
				
				if(autoTimer.Get() > 3 && autoTimer.Get() < 6)
				{
					dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Left Stick Set");
					
					leftStickVec[Y] = .15;
				}
				else if(autoTimer.Get() >= 6 && autoTimer.Get() < 8.25)
				{
					leftStickVec[Y] = .50;
				}
				else
				{
					leftStickVec[Y] = 0;
				}
				
				leftStickVec[X] = 0;
				phi = 0;
				
				//Need to change these values based on center/wheel placement.
				wheel[FL].x = .707 * phi;
				wheel[FL].y = .707 * phi;
				wheel[FR].x = .707 * phi;
				wheel[FR].y = -.707 * phi;
				wheel[BR].x = -.707 * phi;
				wheel[BR].y = -.707 * phi;
				wheel[BL].x = -.707 * phi;
				wheel[BL].y = .707 * phi;
	
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
	
				wheel[FL].curTheta = -(posEncFL.GetVoltage() - FLOFFSET ) / 5 * 2
						* PI;
				wheel[FR].curTheta = -(posEncFR.GetVoltage() - FROFFSET) / 5 * 2
						* PI;
				wheel[BR].curTheta = -(posEncBR.GetVoltage() - BROFFSET)/ 5 * 2
						* PI;
				wheel[BL].curTheta = -(posEncBL.GetVoltage() - BLOFFSET) / 5 * 2
						* PI;
	
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
				
				if(moduleCounter == 1)
		        {
					if (!(wheel[FL].x == 0 && wheel[FL].y == 0)) 
					{
						turnWheelFL.Set(-wheel[FL].turnVel);
						moveWheelFL.Set(wheel[FL].mag);
					} 
					else 
					{
						turnWheelFL.Set(0);
						moveWheelFL.Set(0);
					}
		        }
		        if(moduleCounter == 2)
		        {
					if (!(wheel[FR].x == 0 && wheel[FR].y == 0)) 
					{
						turnWheelFR.Set(-wheel[FR].turnVel);
						moveWheelFR.Set(-wheel[FR].mag);
			
					} 
					else 
					{
						turnWheelFR.Set(0);
						moveWheelFR.Set(0);
					}
		        }
		        if(moduleCounter == 3)
		        {
					if (!(wheel[BL].x == 0 && wheel[BL].y == 0)) 
					{
						turnWheelBL.Set(-wheel[BL].turnVel);		 
						moveWheelBL.Set(wheel[BL].mag);
			
					} 
					else 
					{
						turnWheelBL.Set(0);
						moveWheelBL.Set(0);
					}
		        }
		        if(moduleCounter == 4)
		        {
					if (!(wheel[BR].x == 0 && wheel[BR].y == 0)) 
					{
						turnWheelBR.Set(-wheel[BR].turnVel);			 
						moveWheelBR.Set(wheel[BR].mag);
			
					} 
					else 
					{
						turnWheelBR.Set(0);
						moveWheelBR.Set(0);
					}
		        }
		        
		        for(i=0; i<4; i++)
	    		{
	    			wheel[i].prevTurnVel = wheel[i].turnVel;
	    		}
			} //End of if
			
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
			
			dsLCD->UpdateLCD();
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
				
				leftStickVec[RAWX] = deadBand(stick.GetRawAxis(1));
				leftStickVec[RAWY] = deadBand(stick.GetRawAxis(2));
				leftStickVec[X] = leftStickVec[RAWX]*sqrt(1-.5*pow(
						leftStickVec[RAWY], 2));
				leftStickVec[Y] = -leftStickVec[RAWY]*sqrt(1-.5*pow(
						leftStickVec[RAWX], 2));
				phi = deadBand(stick.GetRawAxis(3)); //Should be right stick x.
				
				if (stick.GetRawButton(5))
				{
					moveVal = .9;
				}
				else
				{
					moveVal = .5;
				}
				
				if (stick.GetRawButton(4)){
					leftStickVec[X] = 0;
					leftStickVec[Y] = moveVal;
				} 
				else if (stick.GetRawButton(3)){
					leftStickVec[X] = moveVal;
					leftStickVec[Y] = 0;
				}
				else if (stick.GetRawButton(2)){
					leftStickVec[X] = 0;
					leftStickVec[Y] = -moveVal;
				}
				else if (stick.GetRawButton(1)){
					leftStickVec[X] = -moveVal;
					leftStickVec[Y] = 0;
				}				
				
				//Need to change these values based on center/wheel placement.
				wheel[FL].x = .707 * phi;
				wheel[FL].y = .707 * phi;
				wheel[FR].x = .707 * phi;
				wheel[FR].y = -.707 * phi;
				wheel[BR].x = -.707 * phi;
				wheel[BR].y = -.707 * phi;
				wheel[BL].x = -.707 * phi;
				wheel[BL].y = .707 * phi;
	
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
	
				wheel[FL].curTheta = -(posEncFL.GetVoltage() - FLOFFSET ) / 5 * 2
						* PI;
				wheel[FR].curTheta = -(posEncFR.GetVoltage() - FROFFSET) / 5 * 2
						* PI;
				wheel[BR].curTheta = -(posEncBR.GetVoltage() - BROFFSET)/ 5 * 2
						* PI;
				wheel[BL].curTheta = -(posEncBL.GetVoltage() - BLOFFSET) / 5 * 2
						* PI;
	
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
				
		        if(moduleCounter == 1)
		        {
					if (!(wheel[FL].x == 0 && wheel[FL].y == 0)) 
					{
						turnWheelFL.Set(-wheel[FL].turnVel);
						moveWheelFL.Set(wheel[FL].mag);
					} 
					else 
					{
						turnWheelFL.Set(0);
						moveWheelFL.Set(0);
					}
		        }
		        if(moduleCounter == 2)
		        {
					if (!(wheel[FR].x == 0 && wheel[FR].y == 0)) 
					{
						turnWheelFR.Set(-wheel[FR].turnVel);
						moveWheelFR.Set(-wheel[FR].mag);
			
					} 
					else 
					{
						turnWheelFR.Set(0);
						moveWheelFR.Set(0);
					}
		        }
		        if(moduleCounter == 3)
		        {
					if (!(wheel[BL].x == 0 && wheel[BL].y == 0)) 
					{
						turnWheelBL.Set(-wheel[BL].turnVel);		 
						moveWheelBL.Set(wheel[BL].mag);
			
					} 
					else 
					{
						turnWheelBL.Set(0);
						moveWheelBL.Set(0);
					}
		        }
		        if(moduleCounter == 4)
		        {
					if (!(wheel[BR].x == 0 && wheel[BR].y == 0)) 
					{
						turnWheelBR.Set(-wheel[BR].turnVel);			 
						moveWheelBR.Set(wheel[BR].mag);
			
					} 
					else 
					{
						turnWheelBR.Set(0);
						moveWheelBR.Set(0);
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
			else if (stick.GetRawButton(8) && !calibrating)
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
				shooterMotor1.Set(.3);
				shooterMotor2.Set(.3);
			}
			else if (manipStick.GetRawButton(1) && !calibrating)
			{
				shooterMotor1.Set(-1.0);
				shooterMotor2.Set(-1.0);
			}
			else
			{
				shooterMotor1.Set(0);
				shooterMotor2.Set(0);
			}
			
			// **************************************
			// ********BEGIN CALIBRATION CODE********
			// **************************************
			
			if (calibrating == true)
			{
				flOff = posEncFL.GetVoltage();
				frOff = posEncFR.GetVoltage();
				brOff = posEncBR.GetVoltage();
				blOff = posEncBL.GetVoltage();
				
				dsLCD->Printf(DriverStationLCD::kUser_Line2, 1,
					"FLOFFSET: %f     ", flOff);
				dsLCD->Printf(DriverStationLCD::kUser_Line3, 1,
					"FROFFSET: %f     ", frOff);
				dsLCD->Printf(DriverStationLCD::kUser_Line5, 1,
					"BLOFFSET: %f     ", blOff);
				dsLCD->Printf(DriverStationLCD::kUser_Line4, 1,
					"BROFFSET: %f     ", brOff);
				if (stick.GetRawButton(8) && !isButtonPressed) 
				{
					if (calMode == 0) 
					{
						flOff = posEncFL.GetVoltage();
					} 
					else if (calMode == 1) 
					{
						frOff = posEncFR.GetVoltage();
					} 
					else if (calMode == 2) 
					{
						blOff = posEncBL.GetVoltage();
					} 
					else if (calMode == 3) 
					{
						brOff = posEncBR.GetVoltage();
					}

					calMode++;
					isButtonPressed = true;
				}

				else if (!stick.GetRawButton(8)) 
				{
					isButtonPressed = false;
				}
				
				dsLCD->Printf(DriverStationLCD::kUser_Line1, 1,
					"**CALIBRATING WHEEL %i", calMode+1);
				
				if (stick.GetRawButton(2) == true && calMode == 0) 
				{
					turnWheelFL.Set(OFFSETMOVE);
				} 
				else if (stick.GetRawButton(3) == true && calMode == 0) 
				{
					turnWheelFL.Set(-OFFSETMOVE);
				} 
				
				else if (stick.GetRawButton(1) == true && calMode == 0) 
				{
					turnWheelFL.Set(OFFSETMOVE/2);
				} 
				else if (stick.GetRawButton(4) == true && calMode == 0) 
				{
					turnWheelFL.Set(-OFFSETMOVE/2);
				} 
				else 
				{
					turnWheelFL.Set(0);
				}

				if (stick.GetRawButton(2) == true && calMode == 1) 
				{
					turnWheelFR.Set(OFFSETMOVE);
				} 
				else if (stick.GetRawButton(3) == true && calMode == 1) 
				{
					turnWheelFR.Set(-OFFSETMOVE);
				} 
				else if (stick.GetRawButton(1) == true && calMode == 1) 
				{
					turnWheelFR.Set(OFFSETMOVE/2);
				} 
				else if (stick.GetRawButton(4) == true && calMode == 1) 
				{
					turnWheelFR.Set(-OFFSETMOVE/2);
				} 
				else 
				{
					turnWheelFR.Set(0);
				}

				if (stick.GetRawButton(2) == true && calMode == 2) 
				{
					turnWheelBL.Set(OFFSETMOVE);
				} 
				else if (stick.GetRawButton(3) == true && calMode == 2) 
				{
					turnWheelBL.Set(-OFFSETMOVE);
				} 
				else if (stick.GetRawButton(1) == true && calMode == 2) 
				{
					turnWheelBL.Set(OFFSETMOVE/2);
				} 
				else if (stick.GetRawButton(4) == true && calMode == 2) 
				{
					turnWheelBL.Set(-OFFSETMOVE/2);
				} 
				else 
				{
					turnWheelBL.Set(0);
				}

				if (stick.GetRawButton(2) == true && calMode == 3) 
				{
					turnWheelBR.Set(OFFSETMOVE);
				} 
				else if (stick.GetRawButton(3) == true && calMode == 3) 
				{
					turnWheelBR.Set(-OFFSETMOVE);
				} 
				else if (stick.GetRawButton(1) == true && calMode == 3) 
				{
					turnWheelBR.Set(OFFSETMOVE/2);
				} 
				else if (stick.GetRawButton(4) == true && calMode == 3) 
				{
					turnWheelBR.Set(-OFFSETMOVE/2);
				} 
				else if (calMode >= 4)
				{
					calMode = 0;
					calibrating = false;
				}
				else 
				{
					turnWheelBR.Set(0);
				}
				
				dsLCD->UpdateLCD();
			}
			
			if (manipStick.GetRawButton(8))
			{
				calibrating = true;
			}
	
		}
	}

/**
 * Runs during test mode
 */
	void Test() {
	
	}
};

START_ROBOT_CLASS(RobotDemo)
;

float deadBand(float axisValue) 
{
	if (axisValue < -.05 || axisValue> .05){
	return axisValue;
	}
	else
	{
		return 0.0;
	}
}

