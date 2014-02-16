#include "WPILib.h"
#include "math.h"
#include "controls.h"
#include "offsets.h"
//#include "util.h"
#define TESTVAL                 .4
#define OFFSETMOVE				.2
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
//#define TESTER                  1
//#define VOLTAGERATE				10000

float deadBand(float);

struct wheelVector 
{
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
	CANJaguar turnWheelFL;
	CANJaguar turnWheelFR;
	CANJaguar turnWheelBR;
	CANJaguar turnWheelBL;
	CANJaguar moveWheelFL;
	CANJaguar moveWheelFR;
	CANJaguar moveWheelBR;
	CANJaguar moveWheelBL;
	CANJaguar pickUpArm1;
	CANJaguar pickUpArm2;
	Victor shooterMotor1;
	Victor shooterMotor2;
	AnalogChannel posEncFL;
	AnalogChannel posEncFR;
	AnalogChannel posEncBR;
	AnalogChannel posEncBL;
	Timer baneTimer;
	//float magmodifier;

public:
	RobotDemo() :
		stick(1), turnWheelFL(4), turnWheelFR(11), turnWheelBR(27),
				turnWheelBL(9), moveWheelFL(2), moveWheelFR(45),
				moveWheelBR(30), moveWheelBL(13), pickUpArm1(12), 
				pickUpArm2(46), shooterMotor1(1), shooterMotor2(2),
				posEncFL(5), posEncFR(1), posEncBR(7), posEncBL(3)
	{
		Watchdog().SetExpiration(1);
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

		float leftStickVec[4];
		float phi;
		float largestMag;
		wheelVector wheel[4];
		int i;
		int j;
		bool calibrating = false;
		int calMode;
		bool isButtonPressed;
		isButtonPressed = false;
		
//		moveWheelFL.ChangeControlMode(CANJaguar::kSpeed);
//		moveWheelFL.ConfigEncoderCodesPerRev(1000);
//		moveWheelFL.SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
//		moveWheelFL.SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
//		moveWheelFL.SetPID(PVALUE,IVALUE,DVALUE);
//		moveWheelFL.EnableControl();
//		Watchdog().Feed();
//		moveWheelFR.ChangeControlMode(CANJaguar::kSpeed);
//		moveWheelFR.ConfigEncoderCodesPerRev(1000);
//		moveWheelFR.SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
//		moveWheelFR.SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
//		moveWheelFR.SetPID(PVALUE,IVALUE,DVALUE);
//		moveWheelFR.EnableControl();
//		Watchdog().Feed();
//		moveWheelBL.ChangeControlMode(CANJaguar::kSpeed);
//		moveWheelBL.ConfigEncoderCodesPerRev(1000);
//		moveWheelBL.SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
//		moveWheelBL.SetPID(PVALUE,IVALUE,DVALUE);
//		moveWheelBL.EnableControl();
//		Watchdog().Feed();
//		moveWheelBR.ChangeControlMode(CANJaguar::kSpeed);
//		moveWheelBR.ConfigEncoderCodesPerRev(1000);
//		moveWheelBR.SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
//		moveWheelBR.SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder	);
//		moveWheelBR.SetPID(PVALUE,IVALUE,DVALUE);
//		moveWheelBR.EnableControl();
//		turnWheelBR.ChangeControlMode(CANJaguar::kPercentVbus);
//		turnWheelBR.ChangeControlMode(CANJaguar::kPercentVbus);
//		turnWheelBR.ChangeControlMode(CANJaguar::kPercentVbus);
//		turnWheelBR.ChangeControlMode(CANJaguar::kPercentVbus);	
		
		for (i = 0; i < 4; i++) {
			wheel[i].changeSign = false;
			wheel[i].prevTurnVel = 0;
		}
		
		/*turnWheelFL.SetVoltageRampRate(voltageRate);
		turnWheelFR.SetVoltageRampRate(voltageRate);
		turnWheelBR.SetVoltageRampRate(voltageRate);
		turnWheelBL.SetVoltageRampRate(voltageRate);*/

		while (IsOperatorControl()) 
		{
			Watchdog().Feed();

			leftStickVec[RAWX] = deadBand(stick.GetRawAxis(1));
			leftStickVec[RAWY] = deadBand(stick.GetRawAxis(2));
			leftStickVec[X] = leftStickVec[RAWX]*sqrt(1-.5*pow(
				leftStickVec[RAWY], 2));
			leftStickVec[Y] = -leftStickVec[RAWY]*sqrt(1-.5*pow(
				leftStickVec[RAWX], 2));
			phi = deadBand(stick.GetRawAxis(3)); //Should be right stick x.
			
			
			if (stick.GetRawButton(5) && stick.GetRawButton(6))
			{
				calibrating = true;
			}
			
			while (calibrating == true)
			{ 
				
				if (stick.GetRawButton(8) && !isButtonPressed) 
				{
					if (calMode == 0) 
					{
						flOffset = posEncFL.GetVoltage();
						dsLCD->Printf(DriverStationLCD::kUser_Line2, 1,
							"OFFSET(%i) SET TO %f     ", calMode+1, flOffset);
					} 
					else if (calMode == 1) 
					{
						frOffset = posEncFR.GetVoltage();
						dsLCD->Printf(DriverStationLCD::kUser_Line3, 1,
							"OFFSET(%i) SET TO %f     ", calMode+1, frOffset);
					} 
					else if (calMode == 2) 
					{
						blOffset = posEncBL.GetVoltage();
						dsLCD->Printf(DriverStationLCD::kUser_Line4, 1,
							"OFFSET(%i) SET TO %f     ", calMode+1, blOffset);
					} 
					else if (calMode == 3) 
					{
						brOffset = posEncBR.GetVoltage();
						dsLCD->Printf(DriverStationLCD::kUser_Line5, 1,
							"OFFSET(%i) SET TO %f     ", calMode+1, brOffset);
					}
	
					calMode++;
					if (calMode >= 4)
					{
						calibrating = false;
					}
					isButtonPressed = true;
				}
				else if (!stick.GetRawButton(8)) 
				{
					isButtonPressed = false;
				}
				
				
				
			}
			
			#ifdef TESTER
				if (stick.GetRawButton(4))
				{
					leftStickVec[X] = 0;
					leftStickVec[Y] = TESTVAL;
				} 
				else if (stick.GetRawButton(3))
				{
					leftStickVec[X] = TESTVAL;
					leftStickVec[Y] = 0;
				}
				else if (stick.GetRawButton(2))
				{
					leftStickVec[X] = 0;
					leftStickVec[Y] = -TESTVAL;
				}
				else if (stick.GetRawButton(1))
				{
					leftStickVec[X] = -TESTVAL;
					leftStickVec[Y] = 0;
				}				
			#endif
			
			//Need to change these values based on center/wheel placement.
			wheel[FL].x = .707 * phi;
			wheel[FL].y = .707 * phi;
			wheel[FR].x = .707 * phi;
			wheel[FR].y = -.707 * phi;
			wheel[BL].x = -.707 * phi;
			wheel[BL].y = -.707 * phi;
			wheel[BR].x = -.707 * phi;
			wheel[BR].y = .707 * phi;

			wheel[FL].x += leftStickVec[X];
			wheel[FL].y += leftStickVec[Y];
			wheel[FR].x += leftStickVec[X];
			wheel[FR].y += leftStickVec[Y];
			wheel[BR].x += leftStickVec[X];
			wheel[BR].y += leftStickVec[Y];
			wheel[BL].x += leftStickVec[X];
			wheel[BL].y += leftStickVec[Y];
			
			for (i = 0; i <= 3; i++) 
			{
				wheel[i].mag = MAXPOWER * sqrt(pow(wheel[i].x, 2) 
					+ pow(wheel[i].y, 2));
			}

			for (i = 0; i <= 3; i++) 
			{
				if (wheel[i].mag > 1 * MAXPOWER) 
				{
					largestMag = wheel[i].mag;
					for (j = 0; j <= 3; j++) 
					{
						wheel[j].mag = MAXPOWER * wheel[j].mag / largestMag;
					}
				}
			}

			for (i = 0; i <= 3; i++) 
			{
				wheel[i].tarTheta = atan(wheel[i].y / wheel[i].x);

				if (wheel[i].x < 0) 
				{
					wheel[i].tarTheta += PI;
				}

			}

			wheel[FL].curTheta = -(posEncFL.GetVoltage() - flOffset ) / 5 * 2
					* PI;
			wheel[FR].curTheta = -(posEncFR.GetVoltage() - frOffset) / 5 * 2
					* PI;
			wheel[BR].curTheta = -(posEncBR.GetVoltage() - blOffset)/ 5 * 2
					* PI;
			wheel[BL].curTheta = -(posEncBL.GetVoltage() - blOffset) / 5 * 2
					* PI;

			for (i=0; i <= 3; i++) 
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
				
				if (0 < wheel[i].turnVel && wheel[i].turnVel < .17)
				{
					wheel[i].turnVel = .17;
				} 
				if (0 > wheel[i].turnVel && wheel[i].turnVel > -.17)
				{
					wheel[i].turnVel = -.17;
				}
				if (fabs(wheel[i].diffTheta) < PI/45 )
				{
					wheel[i].turnVel = 0;
				}
			}
			
			dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Mag: %f        ",
				wheel[FL].mag);
			dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Diff: %f        ",
				wheel[FL].diffTheta);
			dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, 
				"Coords: (%3.2f,%3.2f)        ",wheel[FL].x, wheel[FL].y);
			dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "Turn Val: %f         ", wheel[FL].turnVel);
			dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "EncoderPos: %f        ",
				moveWheelFL.GetPosition());
			dsLCD->UpdateLCD();
			
			for (i = 0; i < 4; i++) 
			{
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
			if (!(wheel[FR].x == 0 && wheel[FR].y == 0)) 
			{
				turnWheelFR.Set(-wheel[FR].turnVel);		 
				moveWheelFR.Set(wheel[FR].mag);
			} 
			else 
			{
				turnWheelFR.Set(0);
				moveWheelFR.Set(0);
			}
			if (!(wheel[BL].x == 0 && wheel[BL].y == 0)) 
			{
				turnWheelBL.Set(-wheel[BL].turnVel);				 
				moveWheelBL.Set(wheel[BL].mag);
			} else 
			{
				turnWheelBL.Set(0);
				moveWheelBL.Set(0);
			}
			if (!(wheel[BR].x == 0 && wheel[BR].y == 0)) 
			{
				turnWheelBR.Set(-wheel[BR].turnVel);
			 
				moveWheelBR.Set(wheel[BR].mag);\
			} 
			else 
			{
				turnWheelBR.Set(0);
				moveWheelBR.Set(0);
			}

			for(i=0; i<4; i++)
			{
				wheel[i].prevTurnVel = wheel[i].turnVel;
			}
//			if (stick.GetRawButton(7))
//			{
//				pickUpArm1.Set(1);
//				pickUpArm2.Set(-1);
//			}
//			else if (stick.GetRawButton(5))
//			{
//				pickUpArm1.Set(-1);
//				pickUpArm2.Set(1);
//			if (stick.GetRawButton(6))
//			{
//				shooterMotor1.Set(1);
//				shooterMotor2.Set(1);
//			}
//			else if (stick.GetRawButton(8))
//			{
//				shooterMotor1.Set(-1);
//				shooterMotor2.Set(-1);
//			}
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

START_ROBOT_CLASS(RobotDemo);

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

