#include "WPILib.h"
#include "math.h"
#include "controls.h"
#include "offsets.h"
//#include "util.h"
#define TESTVAL                 .5
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
#define TESTER                  1
#define STAGGERDELAY			0.005 //In seconds
//#define VOLTAGERATE				10000

float deadBand(float);

struct wheelVector {
	float rawx, x, rawy, y, mag, tarTheta, curTheta, diffTheta, turnVel;

	float prevTurnVel;
	bool changeSign;
	float moveTime;
	float offset;
	
	CANJaguar *turnWheel;
	CANJaguar *moveWheel;
	AnalogChannel *posEncoder;
};

/**
 * This is a demo program showing the use of the RobotBase class.
 * The SimpleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 */
class RobotDemo : public SimpleRobot {
	wheelVector wheel[4];
	Joystick stick; // only joystick
	Joystick manipStick;
	Joystick controlStick;
//	CANJaguar shooterMotor1;
//	CANJaguar shooterMotor2;
	Talon pickUpArm1;
	Talon pickUpArm2;
	Timer staggerTimer;
	Timer baneTimer;
	Timer autoTimer;
	//float magmodifier;

public:
	RobotDemo() :
		stick(1), manipStick(2),controlStick(3),
			//shooterMotor1(13), shooterMotor2(46),
			pickUpArm1(1), pickUpArm2(10)
		{
		Watchdog().SetExpiration(1);
		
		wheel[FL].turnWheel = new CANJaguar(9);
		wheel[FL].moveWheel = new CANJaguar(27);
		wheel[FL].posEncoder = new AnalogChannel(3);
		wheel[FR].turnWheel = new CANJaguar(11);
		wheel[FR].moveWheel = new CANJaguar(4);
		wheel[FR].posEncoder = new AnalogChannel(2);
		wheel[BR].turnWheel = new CANJaguar(45);
		wheel[BR].moveWheel = new CANJaguar(2);
		wheel[BR].posEncoder = new AnalogChannel(4);
		wheel[BL].turnWheel = new CANJaguar(30);
		wheel[BL].moveWheel = new CANJaguar(12);
		wheel[BL].posEncoder = new AnalogChannel(1);
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

		baneTimer.Start();
		staggerTimer.Start();

		float leftStickVec[4];
		float phi;
		float largestMag;
		wheelVector wheel[4];
		int i;
		int j;
		int moduleCounter = 0;
		bool isButtonPressed;
		isButtonPressed = false;	
		
		wheel[FL].offset = 4.200;
		wheel[FR].offset = 1.941;
		wheel[BR].offset = 3.585;
		wheel[BL].offset = 1.268;
		
		for (i = 0; i < 4; i++) {
			wheel[i].changeSign = false;
			wheel[i].prevTurnVel = 0;
		}
		

		while (IsOperatorControl()) {
			Watchdog().Feed();


			
//#define TESTER			1

			leftStickVec[RAWX] = deadBand(stick.GetRawAxis(1));
			leftStickVec[RAWY] = deadBand(stick.GetRawAxis(2));
			leftStickVec[X] = leftStickVec[RAWX]*sqrt(1-.5*pow(
					leftStickVec[RAWY], 2));
			leftStickVec[Y] = -leftStickVec[RAWY]*sqrt(1-.5*pow(
					leftStickVec[RAWX], 2));
			phi = deadBand(stick.GetRawAxis(3)); //Should be right stick x.
			
			#ifdef TESTER
				if (stick.GetRawButton(4)){
					leftStickVec[X] = 0;
					leftStickVec[Y] = TESTVAL;
				} 
				else if (stick.GetRawButton(3)){
					leftStickVec[X] = TESTVAL;
					leftStickVec[Y] = 0;
				}
				else if (stick.GetRawButton(2)){
					leftStickVec[X] = 0;
					leftStickVec[Y] = -TESTVAL;
				}
				else if (stick.GetRawButton(1)){
					leftStickVec[X] = -TESTVAL;
					leftStickVec[Y] = 0;
				}				
			#endif
			
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
			
			
			

			for (i = 0; i < 4; i++) {
				wheel[i].mag = MAXPOWER * sqrt(pow(wheel[i].x, 2) + pow(wheel[i].y, 2));
			}

			for (i = 0; i < 4; i++) {
				if (wheel[i].mag > 1 * MAXPOWER) {
					largestMag = wheel[i].mag;
					for (j = 0; j < 4; j++) {
						wheel[j].mag = MAXPOWER * wheel[j].mag / largestMag;
					}
				}

			}

			for (i = 0; i < 4; i++) {
				wheel[i].tarTheta = atan(wheel[i].y / wheel[i].x);

				if (wheel[i].x < 0) {
					wheel[i].tarTheta += PI;
				}

			}

			wheel[FL].curTheta = -(wheel[FL].posEncoder->GetVoltage() - wheel[FL].offset) / 5 * 2
					* PI;
			wheel[FR].curTheta = -(wheel[FR].posEncoder->GetVoltage() - wheel[FR].offset) / 5 * 2
					* PI;
			wheel[BR].curTheta = -(wheel[BR].posEncoder->GetVoltage() - wheel[BR].offset)/ 5 * 2
					* PI;
			wheel[BL].curTheta = -(wheel[BL].posEncoder->GetVoltage() - wheel[BL].offset) / 5 * 2
					* PI;

//						if (stick.GetRawButton(2) == true) {
//						 turnWheelFL.Set(.15);
//						 } else if (stick.GetRawButton(3) == true) {
//						 turnWheelFL.Set(-.15);
//						 } else {
//						 turnWheelFL.Set(0);
//						 }

			for (i=0; i < 4; i++) {
				wheel[i].diffTheta = wheel[i].tarTheta - wheel[i].curTheta;

				if (wheel[i].diffTheta > PI) {
					wheel[i].diffTheta -= 2*PI;
				} else if (wheel[i].diffTheta < -PI) {
					wheel[i].diffTheta += 2*PI;
				}

				if (wheel[i].diffTheta > PI/2) {
					wheel[i].diffTheta -= PI;
					wheel[i].mag = wheel[i].mag * -1;
				} else if (wheel[i].diffTheta < -PI/2) {
					wheel[i].diffTheta += PI;
					wheel[i].mag = wheel[i].mag * -1;
				}

				wheel[i].turnVel = wheel[i].diffTheta / (PI/2);
				
				if (0 < wheel[i].turnVel && wheel[i].turnVel < .25){
					wheel[i].turnVel = .25;
				} 
				if (0 > wheel[i].turnVel && wheel[i].turnVel > -.25){
					wheel[i].turnVel = -.25;
				}
				if (fabs(wheel[i].diffTheta) < PI/45 ){
					wheel[i].turnVel = 0;
				}
			}
			
			dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Mag: %f        ",
				wheel[FL].mag);
			dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Diff: %f        ",
				wheel[FL].diffTheta);
			dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "Coords: (%3.2f,%3.2f)        ",
				wheel[FL].x, wheel[FL].y);
			dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "Turn Val: %f         ", wheel[FL].turnVel);
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
				if (wheel[i].changeSign) {
					wheel[i].turnVel = 0;
					if (wheel[i].moveTime < baneTimer.Get()) {
						wheel[i].changeSign = false;
					}
				}
				

			}
		
			if (!(wheel[moduleCounter].x == 0 && wheel[moduleCounter].y == 0)) {
				wheel[moduleCounter].turnWheel->Set(-wheel[moduleCounter].turnVel);
				wheel[moduleCounter].moveWheel->Set(wheel[moduleCounter].mag);
	
			} else {
				wheel[moduleCounter].moveWheel->Set(0);
				wheel[moduleCounter].turnWheel->Set(0);
			}
        
        
		if (staggerTimer.Get() > STAGGERDELAY){
			moduleCounter++;
			staggerTimer.Reset();
		}
		if(moduleCounter > 3)
		{
			moduleCounter = 0;
		}
		
		for(i=0; i<4; i++)
		{
			wheel[i].prevTurnVel = wheel[i].turnVel;
		}
		
		if (stick.GetRawButton(8))
		{
			pickUpArm1.Set(-.8);
			pickUpArm2.Set(-.8);
		}
		else if (stick.GetRawButton(6))
		{
			pickUpArm1.Set(.8);
			pickUpArm2.Set(.8);
		}
		else
		{
			pickUpArm1.Set(0);
			pickUpArm2.Set(0);
		}
		
		/*         Shooter
		if (manipStick.GetRawButton(3))
		{
			shooterMotor1.Set(.3);
			shooterMotor2.Set(.3);
		}
		else if (manipStick.GetRawButton(1))
		{
			shooterMotor1.Set(-1.0);
			shooterMotor2.Set(-1.0);
		}
		else
		{
			shooterMotor1.Set(0);
			shooterMotor2.Set(0);
		}*/

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

