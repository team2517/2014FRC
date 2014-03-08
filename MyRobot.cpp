#include "WPILib.h"
#include "math.h"
#include "Vision/BinaryImage.h"
#include "Math.h"

#define OFFSETMOVE					.25
#define TESTENCODERNUM				7
#define FL                        0
#define FR                        1
#define BR                        2
#define BL                        3
#define X                        0
#define Y                        1
#define PI                        3.1415926535

struct wheelVector 
{
	float x, y, mag, tarTheta, curTheta, turnVel;
};

/**
 * This is a demo program showing the use of the RobotBase class.
 * The SimpleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 */
class RobotDemo : public SimpleRobot 
{
	AnalogChannel posEncFL;
	AnalogChannel posEncFR;
	AnalogChannel posEncBR;
	AnalogChannel posEncBL;
	CANJaguar turnWheelFL;
	CANJaguar turnWheelFR;
	CANJaguar turnWheelBR;
	CANJaguar turnWheelBL;
	Joystick stick;
	float motorspeed;

public:
	RobotDemo() :
		posEncFL(3), posEncFR(2), posEncBR(4), posEncBL(1), 
		turnWheelFL(9), turnWheelFR(11), turnWheelBR(45),
		turnWheelBL(30), stick(1) 
	{
		Watchdog().SetExpiration(1);
	}

	/**
	 * Drive left & right motors for 2 seconds then stop
	 */
	void Autonomous() 
	{

	}

	/**
	 * Runs the motors with arcade steering. 
	 */
	void OperatorControl() 
	{
		Watchdog().SetEnabled(true);
		
		bool isButtonPressed = false;
		int calMode = 0;
		float flOffset;
		float frOffset;
		float brOffset;
		float blOffset;

		DriverStationLCD *dsLCD = DriverStationLCD::GetInstance();

		while (IsOperatorControl()) 
		{
			Watchdog().Feed();
			
			flOffset = posEncFL.GetVoltage();
			frOffset = posEncFR.GetVoltage();
			brOffset = posEncBR.GetVoltage();
			blOffset = posEncBL.GetVoltage();
			
			dsLCD->Printf(DriverStationLCD::kUser_Line2, 1,
				"flOffset SET TO %f     ", flOffset);
			dsLCD->Printf(DriverStationLCD::kUser_Line3, 1,
				"frOffset SET TO %f     ", frOffset);
			dsLCD->Printf(DriverStationLCD::kUser_Line4, 1,
				"brOffset SET TO %f     ", brOffset);
			dsLCD->Printf(DriverStationLCD::kUser_Line5, 1,
				"blOffset SET TO %f     ", blOffset);

			if (stick.GetRawButton(8) && !isButtonPressed) 
			{
				if (calMode == 0) 
				{
					flOffset = posEncFL.GetVoltage();
//					dsLCD->Printf(DriverStationLCD::kUser_Line2, 1,
//						"OFFSET(%i) SET TO %f     ", calMode+1, flOffset);
				} 
				else if (calMode == 1) 
				{
					frOffset = posEncFR.GetVoltage();
//					dsLCD->Printf(DriverStationLCD::kUser_Line3, 1,
//						"OFFSET(%i) SET TO %f     ", calMode+1, frOffset);
				} 
				else if (calMode == 2) 
				{
					blOffset = posEncBL.GetVoltage();
//					dsLCD->Printf(DriverStationLCD::kUser_Line4, 1,
//						"OFFSET(%i) SET TO %f     ", calMode+1, blOffset);
				} 
				else if (calMode == 3) 
				{
					brOffset = posEncBR.GetVoltage();
//					dsLCD->Printf(DriverStationLCD::kUser_Line5, 1,
//						"OFFSET(%i) SET TO %f     ", calMode+1, brOffset);
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
			}
			else 
			{
				turnWheelBR.Set(0);
			}
			
			dsLCD->UpdateLCD();

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

