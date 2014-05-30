#include "WPILib.h"
#include "math.h"
#include "offsets.h"
#include "SwerveDrive.hpp"
#define MAXPOWER				1
#define STAGGERDELAY			0.005 //In seconds
#define OFFSETMOVE				.25
#define IDLESPEED				0.0

float deadBand(float);

class RobotDemo : public SimpleRobot {
	Joystick manipStick;
	Joystick driveStick;
	Talon pickUpArm1;
	Talon pickUpArm2;
	SwerveDrive *drive;
//	SwerveModule *moduleFL;
	
	
public:
	RobotDemo() :
		manipStick(2), driveStick(1), pickUpArm1(1),pickUpArm2(10)
	{
		Watchdog().SetExpiration(1);
		
		
		
		/* * *
		 * 	Jag Ids
		 * 27 = moveFL
		 * 9  = turnFL
		 * 4  = moveFR
		 * 11 = turnFR
		 * 2  = moveBR
		 * 45 = turnBR
		 * 30 = turnBL
		 * 13 = moveBL
		 * 12 = null
		 * * */
		
		modulePlug plugsFL;
		plugsFL.analogChannel = 3;
		plugsFL.turnID = 9;
		plugsFL.moveID = 27;
		plugsFL.xRotation = .707;
		plugsFL.yRotation = .707;
		plugsFL.offset = FLOFFSET;
		modulePlug plugsFR;
		plugsFR.analogChannel = 2;
		plugsFR.turnID = 11; 
		plugsFR.moveID = 4;
		plugsFR.xRotation = .707;
		plugsFR.yRotation = -.707;
		plugsFR.offset = FROFFSET;
		modulePlug plugsBR;
		plugsBR.analogChannel = 4;
		plugsBR.turnID = 45;
		plugsBR.moveID = 2;
		plugsBR.xRotation = -.707;
		plugsBR.yRotation = -.707;
		plugsBR.offset = BROFFSET;
		modulePlug plugsBL;
		plugsBL.analogChannel = 5;
		plugsBL.turnID = 30;
		plugsBL.moveID = 13;
		plugsBL.xRotation = -.707;
		plugsBL.yRotation = .707;
		plugsBL.offset = BLOFFSET;
		
		drive = new SwerveDrive(plugsFL, plugsFR, plugsBR, plugsBL);
//		moduleFL = new SwerveModule(plugsFL);
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
	void OperatorControl() {
		Watchdog().SetEnabled(true);
		float lastMag;
		
		while (IsOperatorControl()) 
		{
			Watchdog().Feed();
			
			drive->moveDrive(deadBand(driveStick.GetRawAxis(1)), 
				 deadBand(driveStick.GetRawAxis(2)), 
				 deadBand(driveStick.GetRawAxis(3)));
			
//			lastMag = moduleFL->getMagnitude(deadBand(-driveStick.GetRawAxis(1)), 
//								   deadBand(driveStick.GetRawAxis(2)), 
//								   deadBand(driveStick.GetRawAxis(3)));
//
//			if (lastMag > 1) //Make sure lastMag isn't greater than 1
//			{
//				lastMag = lastMag/lastMag;
//			}
//			
//			moduleFL->setSpeed(lastMag);
//			
			if (driveStick.GetRawButton(6))
			{
				pickUpArm1.Set(.8);
				pickUpArm2.Set(.8);
			}
			else if (driveStick.GetRawButton(5))
			{
				pickUpArm1.Set(-.8);
				pickUpArm2.Set(-.8);
			}
			else
			{
				pickUpArm1.Set(0);
				pickUpArm2.Set(0);
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
	if (axisValue < -.1 || axisValue> .1){ 
	return axisValue;
}
else
{
	return 0.0;
}
}

