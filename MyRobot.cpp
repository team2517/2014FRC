#include "WPILib.h"
#include "math.h"
#include "Vision/BinaryImage.h"
#include "Math.h"
#include "controls.h"
#include "offsets.h"
//#include "util.h"

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
		DriverStationLCD *dsLCD = DriverStationLCD::GetInstance();
		ParticleFilterCriteria2 criteria[] = { { IMAQ_MT_AREA, 500,
						65535, false, false } }; //Particle filter criteria, used to filter out small particles
		AxisCamera &camera = AxisCamera::GetInstance("10.25.17.11"); //To use the Axis camera uncomment this line
		
		Watchdog().SetEnabled(true);

				while (IsAutonomous() && IsEnabled()) {

					Watchdog().Feed();
					/**
					 * Do the image capture with the camera and apply the algorithm described above. This
					 * sample will either get images from the camera or from an image file stored in the top
					 * level directory in the flash memory on the cRIO. The file name in this case is "testImage.jpg"
					 */
					ColorImage *image;

					image = camera.GetImage(); //To get the images from the camera comment the line above and uncomment this one
					BinaryImage *thresholdImage = image->ThresholdRGB(0, 78, 144, 255,
							86, 255); // get just the green target pixels
					BinaryImage *convexHullImage = thresholdImage->ConvexHull(false); // fill in partial and full rectangles
					BinaryImage *filteredImage = convexHullImage->ParticleFilter(
					 criteria, 1); //Remove small particles

					vector<ParticleAnalysisReport> *reports =
							filteredImage->GetOrderedParticleAnalysisReports(); //get a particle analysis report for each particle
					
					dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Penguins = %i",
						reports->size());
					
					
					if(reports->size())
					{
						ParticleAnalysisReport *report = &(reports->at(0));
						dsLCD->Printf(DriverStationLCD::kUser_Line2, 2, "x = %i, y = %i", report->center_mass_x, report->center_mass_y);
					}
					else
					{
						dsLCD->Printf(DriverStationLCD::kUser_Line2, 2, "Nuttin");
					}
					//dsLCD->Printf(DriverStationLCD::kUser_Line2, 2, "Test");
					dsLCD->UpdateLCD();

					// be sure to delete images after using them
					delete filteredImage;
					delete convexHullImage;
					delete thresholdImage;
					delete image;

					//delete allocated reports and Scores objects also
					delete reports;			
				}
	}

	/**
	 * Runs the motors with arcade steering. 
	 */
	void OperatorControl() {
		Watchdog().SetEnabled(true);

		float leftStickVec[2];
		float phi;
		float largestMag;
		wheelVector wheel[4];
		int i;
		int j;

		while (IsOperatorControl()) {
			Watchdog().Feed();

			leftStickVec[X] = deadBand(stick.GetRawAxis(1));
			leftStickVec[Y] = deadBand(stick.GetRawAxis(2));
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
				wheel[i].mag = sqrt(pow(wheel[i].x, 2) + pow(wheel[i].y, 2));
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

			wheel[FL].curTheta = (posEncFL.GetVoltage() - FLOFFSET   ) / 5 * 2 * PI;
			wheel[FR].curTheta = (posEncFR.GetVoltage() - FROFFSET) / 5 * 2 * PI;
			wheel[BR].curTheta = (posEncBR.GetVoltage() - BROFFSET)/ 5 * 2 * PI;
			wheel[BL].curTheta = (posEncBL.GetVoltage() - BLOFFSET) / 5 * 2 * PI;

			for (i=0; i <= 3; i++) {
				wheel[i].tarTheta -= wheel[i].curTheta;

				if (wheel[i].tarTheta > PI) {
					wheel[i].tarTheta -= 2*PI;
				} else if (wheel[i].tarTheta < -PI) {
					wheel[i].tarTheta += 2*PI;
				}

				if (wheel[i].tarTheta > PI/4) {
					wheel[i].tarTheta -= PI/2;
					wheel[i].mag = wheel[i].mag * -1;
				} else if (wheel[i].tarTheta < PI/4) {
					wheel[i].tarTheta += PI/2;
					wheel[i].mag = wheel[i].mag * -1;
				}

				wheel[i].turnVel = wheel[i].tarTheta / PI;
			}

			turnWheelFL.Set(wheel[FL].turnVel);
			turnWheelFR.Set(wheel[FR].turnVel);
			turnWheelBR.Set(wheel[BR].turnVel);
			turnWheelBL.Set(wheel[BL].turnVel);
			moveWheelFL.Set(wheel[FL].mag);
			moveWheelFR.Set(wheel[FR].mag);
			moveWheelBR.Set(wheel[BR].mag);
			moveWheelBL.Set(wheel[BL].mag);

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
	if(axisValue < -.05 || axisValue > .05)
	{
		return axisValue;
	}
	else
	{
		return 0.0;
	}
}


