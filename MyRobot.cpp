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
#define rawX					2
#define rawY					3
#define PI                        3.1415926535

float deadBand(float);

struct wheelVector {
	float rawx, x, rawy, y, mag, tarTheta, curTheta, diffTheta, turnVel;
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
	AnalogChannel posEncFL;
	AnalogChannel posEncFR;
	AnalogChannel posEncBR;
	AnalogChannel posEncBL;
	float magmodifier;

public:
	RobotDemo() :
		stick(1), turnWheelFL(11), turnWheelFR(7), turnWheelBR(8),
				turnWheelBL(9), moveWheelFL(10), moveWheelFR(11),
				moveWheelBR(12), moveWheelBL(13), posEncFL(1), posEncFR(2),
				posEncBR(3), posEncBL(4) {
		Watchdog().SetExpiration(1);
	}

	/**
	 * Drive left & right motors for 2 seconds then stop
	 */
	void Autonomous() {

		while (IsAutonomous() && IsEnabled()) {

		}
	}

	/**
	 * Runs the motors with arcade steering. 
	 */
	void OperatorControl() {
		Watchdog().SetEnabled(true);
		DriverStationLCD *dsLCD = DriverStationLCD::GetInstance();

		float leftStickVec[4];
		float phi;
		float largestMag;
		wheelVector wheel[4];
		int i;
		int j;

		while (IsOperatorControl()) {
			Watchdog().Feed();
			
			
			leftStickVec[rawX] = deadBand(stick.GetRawAxis(1));
			leftStickVec[rawY] = deadBand(stick.GetRawAxis(2));
			leftStickVec[X] = leftStickVec[rawX]*sqrt(1-.5*pow(leftStickVec[rawY], 2));
			leftStickVec[Y] = leftStickVec[rawY]*sqrt(1-.5*pow(leftStickVec[rawX], 2));
			phi = deadBand(stick.GetRawAxis(3)); //Should be right stick x.


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
			wheel[BL].x += leftStickVec[X];
			wheel[BL].y += leftStickVec[Y];
			wheel[BR].x += leftStickVec[X];
			wheel[BR].y += leftStickVec[Y];

			for (i = 0; i <= 3; i++) {
				//wheel[i].x;
				wheel[i].mag = sqrt(pow(wheel[i].x, 2) + pow(wheel[i].y, 2));
				/*if (wheel[i].mag> 1.0) {
				 wheel[i].mag=1;
				 }*/
			}



			for (i = 0; i <= 3; i++) {
				if (wheel[i].mag > 1) {
					largestMag = wheel[i].mag;
					for (j = 0; j <= 3; j++) {
						wheel[j].mag = wheel[j].mag / largestMag;
					}
				}

			}

			for (i = 0; i <= 3; i++) {
				wheel[i].tarTheta = atan(wheel[i].y / wheel[i].x);

				if (wheel[i].x < 0) {
					wheel[i].tarTheta += PI;
				}
				
			}

			wheel[FL].curTheta = (posEncFL.GetVoltage() - FLOFFSET ) / 5 * 2
					* PI;
			wheel[FR].curTheta = (posEncFR.GetVoltage() - FROFFSET) / 5 * 2
					* PI;
			wheel[BR].curTheta = (posEncBR.GetVoltage() - BROFFSET)/ 5 * 2 * PI;
			wheel[BL].curTheta = (posEncBL.GetVoltage() - BLOFFSET) / 5 * 2
					* PI;

			/*if (stick.GetRawButton(2) == true) {
			 turnWheelFL.Set(.15);
			 } else if (stick.GetRawButton(3) == true) {
			 turnWheelFL.Set(-.15);
			 } else {
			 turnWheelFL.Set(0);
			 }*/

			for (i=0; i <= 3; i++) {
				wheel[i].diffTheta = wheel[i].tarTheta - wheel[i].curTheta;

				if (wheel[i].diffTheta > PI) {
					wheel[i].diffTheta -= 2*PI;
				} else if (wheel[i].diffTheta < -PI) {
					wheel[i].diffTheta += 2*PI;
				}
				/*
				 if (wheel[i].tarTheta > PI/4) {
				 wheel[i].tarTheta -= PI/2;
				 wheel[i].mag = wheel[i].mag * -1;
				 } else if (wheel[i].tarTheta < PI/4) {
				 wheel[i].tarTheta += PI/2;
				 wheel[i].mag = wheel[i].mag * -1;
				 }*/

				wheel[i].turnVel = wheel[i].diffTheta / PI;
			}

			
			/*dsLCD->Printf(DriverStationLCD::kUser_Line1, 1,
					"(%f,%f)  ",leftStickVec[X],leftStickVec[Y]); 
			dsLCD->Printf(DriverStationLCD::kUser_Line2, 1,
					"Magnitude: %f",wheel[FL].mag); */
			dsLCD->Printf(DriverStationLCD::kUser_Line1, 1,
					"diffT = %f         ", wheel[FL].diffTheta);
			dsLCD->UpdateLCD();

			if (!(wheel[FL].x == 0 && wheel[FL].y == 0)) {
				turnWheelFL.Set(wheel[FL].turnVel);
				/*turnWheelFR.Set(wheel[FR].turnVel);
				 turnWheelBR.Set(wheel[BR].turnVel);
				 turnWheelBL.Set(wheel[BL].turnVel);
				 
				 moveWheelFL.Set(wheel[FL].mag);
				 moveWheelFR.Set(wheel[FR].mag);
				 moveWheelBR.Set(wheel[BR].mag);
				 moveWheelBL.Set(wheel[BL].mag);
				 */
			}
			else
			{
				turnWheelFL.Set(0);
				/*
				turnWheelFR.Set(0);
				turnWheelBR.Set(0);
				turnWheelBL.Set(0);*/
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

float deadBand(float axisValue) {
	if (axisValue < -.05 || axisValue> .05){
	return axisValue;
}
else
{
	return 0.0;
}
}

