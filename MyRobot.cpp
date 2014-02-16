#include "WPILib.h"
#include "math.h"
#include "Vision/BinaryImage.h"
#include "Math.h"

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
class RobotDemo : public SimpleRobot {
	AnalogChannel testEncoder;
	Joystick stick;
	CANJaguar jaguar;
	float motorspeed;
	
	


public:
	RobotDemo() :
		testEncoder(TESTENCODERNUM), stick(1),jaguar(27){
		Watchdog().SetExpiration(1);
	}

	/**
	 * Drive left & right motors for 2 seconds then stop
	 */
	void Autonomous() {
		
	}

	/**
	 * Runs the motors with arcade steering. 
	 */
	void OperatorControl() {
		Watchdog().SetEnabled(true);

		DriverStationLCD *dsLCD = DriverStationLCD::GetInstance();
		

		while (IsOperatorControl()) {
			Watchdog().Feed();
			
			dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Stick1 = (%.2f,%.2f)   ", stick.GetRawAxis(1), stick.GetRawAxis(2));
			
			if (stick.GetRawButton(2) == true) {
				jaguar.Set(.3);
			}
			else if (stick.GetRawButton(3) == true) {
				jaguar.Set(-.3);
			}
			else{
				jaguar.Set(0);
			}
			
			
			
			

			dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Encoder = %f          ", testEncoder.GetVoltage());
			dsLCD->UpdateLCD();
			Watchdog().Feed();
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

