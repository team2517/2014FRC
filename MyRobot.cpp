#include "WPILib.h"
#include "math.h"
#include "controls.h"
#include "offsets.h"
//#include "util.h"
#define XROTCOMP				.707
#define YROTCOMP				.707
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
#define STALLTHERESHOLD			0
//#define TESTER                  1
//#define VOLTAGERATE				10000

float deadBand(float);

struct wheelVector 
{
	float rawx, x, rawy, y, mag, tarTheta, curTheta, diffTheta, turnVel, prevTheta;
	
	float prevTurnVel;
	bool disable;
	bool changeSign;
	float moveTime;
	
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
	CANJaguar pickUpArm1;
	CANJaguar pickUpArm2;
	Victor shooterMotor1;
	Victor shooterMotor2;
	Timer baneTimer;
	//float magmodifier;

public:
	RobotDemo() :
		stick(1), pickUpArm1(12), pickUpArm2(46), shooterMotor1(1), shooterMotor2(2)
	{
		Watchdog().SetExpiration(1);
		wheel[FL].turnWheel = new CANJaguar(4);
		wheel[FL].moveWheel = new CANJaguar(2);
		wheel[FL].posEncoder = new AnalogChannel(5);
		wheel[FR].turnWheel = new CANJaguar(11);
		wheel[FR].moveWheel = new CANJaguar(45);
		wheel[FR].posEncoder = new AnalogChannel(1);
		wheel[BR].turnWheel = new CANJaguar(27);
		wheel[BR].moveWheel = new CANJaguar(30);
		wheel[BR].posEncoder = new AnalogChannel(7);
		wheel[BL].turnWheel = new CANJaguar(9);
		wheel[BL].moveWheel = new CANJaguar(13);
		wheel[BL].posEncoder = new AnalogChannel(3);
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
		dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Robot Enabled.          ");

		baneTimer.Start();

		float leftStickVec[4];
		float phi;
		float largestMag;
		int i;
		int j;
		bool calibrating = false;
		int calMode;
		bool isButtonPressed;
		bool isThetaCalculated;
		isThetaCalculated = false;
		isButtonPressed = false;
		
//		moveWheelFL.ChangeControlMode(CANJaguar::kSpeed);
//		moveWheelFL.SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
//		moveWheelFL.SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
//		moveWheelFL.ConfigEncoderCodesPerRev(1000);
//		moveWheelFL.SetPID(PVALUE,IVALUE,DVALUE);
//		moveWheelFL.EnableControl();
//		Watchdog().Feed();
//		moveWheelFR.ChangeControlMode(CANJaguar::kSpeed);
//		moveWheelFR.SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
//		moveWheelFR.SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
//		moveWheelFR.ConfigEncoderCodesPerRev(1000);
//		moveWheelFR.SetPID(PVALUE,IVALUE,DVALUE);
//		moveWheelFR.EnableControl();
//		Watchdog().Feed();
//		moveWheelBL.ChangeControlMode(CANJaguar::kSpeed);
//		moveWheelBL.SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
//		moveWheelBL.SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
//		moveWheelBL.ConfigEncoderCodesPerRev(1000);
//		moveWheelBL.SetPID(PVALUE,IVALUE,DVALUE);
//		moveWheelBL.EnableControl();
//		Watchdog().Feed();
//		moveWheelBR.ChangeControlMode(CANJaguar::kSpeed);
//		moveWheelBR.SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
//		moveWheelBR.SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder	);
//		moveWheelBR.ConfigEncoderCodesPerRev(1000);
//		moveWheelBR.SetPID(PVALUE,IVALUE,DVALUE);
//		moveWheelBR.EnableControl();
//		turnWheelBR.ChangeControlMode(CANJaguar::kPercentVbus);
//		turnWheelBR.ChangeControlMode(CANJaguar::kPercentVbus);
//		turnWheelBR.ChangeControlMode(CANJaguar::kPercentVbus);
//		turnWheelBR.ChangeControlMode(CANJaguar::kPercentVbus);	
		
		for (i = 0; i < 4; i++) {
			wheel[i].changeSign = false;
			wheel[i].prevTurnVel = 0;
			wheel[i].disable = false;
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
			
			
			
			if (stick.GetRawButton(9)){
				for (i = 0; i <= 3; i++){
					wheel[i].disable = false;
				}
				
			}
			
			
			if (stick.GetRawButton(10))
			{
				calibrating = true;
			}
			
			if (calibrating == true && IsOperatorControl())
						{ 
							Watchdog().Feed();
							
							if (stick.GetRawButton(8) && !isButtonPressed) 
							{
								if (calMode == 0) 
								{
									//flOffset = posEncFL.GetVoltage();
									flOffset = wheel[FL].posEncoder->GetVoltage();
									dsLCD->Printf(DriverStationLCD::kUser_Line2, 1,
										"OFFSET(%i) SET TO %f     ", calMode+1, flOffset);
								} 
								else if (calMode == 1) 
								{
									frOffset = wheel[FR].posEncoder->GetVoltage();
									dsLCD->Printf(DriverStationLCD::kUser_Line3, 1,
										"OFFSET(%i) SET TO %f     ", calMode+1, frOffset);
								} 
								else if (calMode == 2) 
								{
									brOffset = wheel[BR].posEncoder->GetVoltage();
									dsLCD->Printf(DriverStationLCD::kUser_Line4, 1,
										"OFFSET(%i) SET TO %f     ", calMode+1, blOffset);
								} 
								else if (calMode == 3) 
								{
									brOffset = wheel[BL].posEncoder->GetVoltage();
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
							
							dsLCD->Printf(DriverStationLCD::kUser_Line1, 1,
									"Calibrating Wheel %i          ", calMode+1);
							
							if(stick.GetRawButton(2))
							{
								wheel[calMode].turnWheel->Set(OFFSETMOVE);
							}
							else if (stick.GetRawButton(3))
							{
								wheel[calMode].turnWheel->Set(-OFFSETMOVE);
							}
							else
							{
								wheel[calMode].turnWheel->Set(0);
							}
							
							if(calMode > 3)
							{
								calMode = 0;
							}
										
						}
			
			else
			{
				
				#ifdef TESTER
					if (stick.GetRawButton(4))
					{
						fsakld;jkal;sjdf al;kjfnd m,xcnz.,mvn'';
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
				wheel[FL].x = XROTCOMP * phi;
				wheel[FL].y = YROTCOMP * phi;
				wheel[FR].x = XROTCOMP * phi;
				wheel[FR].y = -YROTCOMP * phi;
				wheel[BL].x = -XROTCOMP * phi;
				wheel[BL].y = -YROTCOMP * phi;
				wheel[BR].x = -XROTCOMP * phi;
				wheel[BR].y = YROTCOMP * phi;
	
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
	
				if (isThetaCalculated == true){
					wheel[FL].prevTheta = wheel[FL].curTheta;
					wheel[FR].prevTheta = wheel[FR].curTheta;
					wheel[BL].prevTheta = wheel[BL].curTheta;
					wheel[BR].prevTheta = wheel[BR].curTheta;
				}
				
				for(i=0; i<4; i++)
				{
					wheel[i].curTheta = -(wheel[i].posEncoder->GetVoltage() 
						- flOffset ) / 5 * 2 * PI;
				}
				
				isThetaCalculated = true;

	
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
				
				dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Mag: %f        ",
					wheel[FL].mag);
				dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "Diff: %f        ",
					wheel[FL].diffTheta);
				dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, 
					"Coords: (%3.2f,%3.2f)        ",wheel[FL].x, wheel[FL].y);
				dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "Turn Val: %f         ", wheel[FL].turnVel);
				dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "EncoderPos: %f        ",
					wheel[FL].moveWheel->GetPosition());
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
				
				for (i = 0; i <= 3; i++)
				{
					if (wheel[i].curTheta - wheel[i].prevTheta 
						< STALLTHERESHOLD && fabs(wheel[i].turnVel) 
						> .17 ){
						dsLCD->Printf(DriverStationLCD::kUser_Line1, 
							1, "M%i STALLED          ",i+1);
						wheel[i].disable = true;
					}
				}
				
				for(i=0;i<4;i++)
				{
					if (!(wheel[i].x == 0 && wheel[i].y == 0 && wheel[i].disable != true))
					{
						wheel[i].turnWheel->Set(-wheel[i].turnVel);
						wheel[i].moveWheel->Set(wheel[i].mag);
					}
					else 
					{
						wheel[i].turnWheel->Set(0);
						wheel[i].moveWheel->Set(0);
					}
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
	//			}
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

