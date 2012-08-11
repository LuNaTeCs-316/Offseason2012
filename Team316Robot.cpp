#include <math.h>
#include "WPILib.h"

// TODO Make sure all of the code is well commented

// 
// This is the class for our robot. It builds off of the IterativeRobot
// class.
//
class Team316Robot : public IterativeRobot
{
private:
	DriverStation *ds;
	
	// Joysticks
	
	Joystick *driverStick;		// XBox controller for driver
	Joystick *operatorStick;	// Logitech joystick for operator
	
	// Motors
	
	RobotDrive *driveMotors; 	// robot drive system
	Jaguar *shooterMotor;		// motor for shooter
	Jaguar *turretMotor;		// motor for turret rotation
	
	// Solenoids
	
	Compressor *compressor;		// the compressor
	Solenoid *upperBallUp;		// upper ball lift solenoid (up channel)
	Solenoid *upperBallDown;	// upper ball lift solenoid (down channel)
	Solenoid *lowerBallUp;		// lower ball lift solenoid (up channel)
	Solenoid *lowerBallDown;	// lower ball lift solenoid (down channel)
	Solenoid *bridgeUp;			// bridge solenoid (up channel)
	Solenoid *bridgeDown;		// bridge solenoid (down channel)
	
	// Relays
	
	Relay *ballPickup;			// relay for ball pickup belt
	
	// Counters
	
	Counter *speedCounter;		// shooter speed counter
	
	// Digitial Inputs
	
	DigitalInput *turretLimitLeft;		// left limit switch for turret
	DigitalInput *turretLimitRight;		// right limit switch for turret
	DigitalInput *ballLoad;				// optical sensor for ball lift
	
	// Global data
	
	int autoMode;		// which autonomous mode are we running?
	int autoStep;		// which step in autonomous mode are we in?
	double startTime;	// used to record the starting time in autonomous
	
	// Camera data
	
	int maxWidth;
	int targetWd;
	int targetHt;
	int targetX;
	int targetY;
	
public:
	//
	// Team316Robot()
	//
	// The only constructor; allocates memory for the dynamic member variables.
	//
	Team316Robot(void)
	{
		ds = DriverStation::GetInstance();
		
		driverStick = new Joystick(1);
		operatorStick = new Joystick(2);
		
		driveMotors = new RobotDrive(1, 2);
		driveMotors->SetExpiration(0.75);
		shooterMotor = new Jaguar(5);
		turretMotor = new Jaguar(6);
		
		compressor = new Compressor(14, 1);
		upperBallUp = new Solenoid(1);
		upperBallDown = new Solenoid(2);
		lowerBallUp = new Solenoid(3);
		lowerBallDown = new Solenoid(4);
		bridgeUp = new Solenoid(5);
		bridgeDown = new Solenoid(6);
		
		ballPickup = new Relay(2);
		
		speedCounter = new Counter(8);
		
		turretLimitLeft = new DigitalInput(4);
		turretLimitRight = new DigitalInput(5);
		ballLoad = new DigitalInput(3);
	}
	
	//
	// RobotInit()
	//
	// Initialize the robot state
	//
	void RobotInit(void)
	{
		// Setup compressor and solenoids
		
		compressor->Start();
		
		upperBallUp->Set(false);
		upperBallDown->Set(true);
		lowerBallUp->Set(true);
		lowerBallDown->Set(false);
		bridgeUp->Set(false);
		bridgeDown->Set(true);
		
		// Start the counter
		
		speedCounter->Start();
		
		// Setup the camera
		
		AxisCamera &camera = AxisCamera::GetInstance();
		camera.WriteResolution(AxisCamera::kResolution_320x240);
		camera.WriteCompression(40);	// might want to try 10
		camera.WriteBrightness(50);		// might want to try lowering
	}

	//
	// DisabledInit()
	//
	// Called once upon entering a disabled state. All outputs should be
	// stopped here for safety.
	//
	void DisabledInit()
	{
		// Put any tasks here to be run when the robot is disabled
	}
	
	//
	// DisabledPeriodic()
	//
	// Called at a regular interval while the robot is disabled. Place tasks
	// such as updating/resetting sensor values
	//
	void DisablePeriodic() {}	// Nothing to be done here for now
	
	//
	// TeleopInit()
	//
	// Called once when entering teleop mode. Setup for teleop mode.
	//
	void TeleopInit()
	{
		compressor->Start();
	}
	
	//
	// TeleopPeriodic()
	//
	// Called at a regular interval (approx. 20ms) during teleop mode.
	// The majority of the teleop code goes here. It is important to
	// make sure this function is not delayed, so don't place any
	// wait statements or loops here.
	//
	void TeleopPeriodic()
	{
		// TODO Divide teleop into functions for each section
		
		// Drive Motors
		
		const float DEADBAND = 0.2;
		
		float drive_x = driverStick->GetX();
		float drive_y = driverStick->GetY();
		float drive_rot = driverStick->GetAxis(Joystick::kTwistAxis);
		
		if (fabs(drive_x) < DEADBAND)
		{
			drive_x = 0;
		}
		if (fabs(drive_y) < DEADBAND)
		{
			drive_y = 0;
		}
		if (fabs(drive_rot) < DEADBAND)
		{
			drive_rot = 0;
		}
		
		driveMotors->MecanumDrive_Cartesian(drive_x, drive_y, drive_rot);
		
		// Turret control
		
		float turretVal = operatorStick->GetX();
		turretVal *= -0.5;
		
		if (operatorStick->GetRawButton(2))
		{
			/*
			FindTarget();
			error = 160 - targetX;
			turretVal = error * kScaling;
			*/
		}
		
		if (turretLimitLeft->Get() && turretVal > 0)
		{
			turretVal = 0;
		}
		else if (turretLimitRight->Get() && turretVal < 0)
		{
			turretVal = 0;
		}
		
		turretMotor->Set(turretVal);
		
		// Ball pickup
		
		if (operatorStick->GetRawButton(2))
		{
			ballPickup->Set(Relay::kForward);
		}
		else
		{
			ballPickup->Set(Relay::kOff);
		}
		
		// Ball loading/firing
		
		if (operatorStick->GetRawButton(4))
		{
			lowerBallDown->Set(true);
			lowerBallUp->Set(false);
		}
		else if (operatorStick->GetRawButton(5) || ballLoad->Get())
		{
			lowerBallUp->Set(true);
			lowerBallDown->Set(false);
		}
		
		if (operatorStick->GetRawButton(1))
		{
			upperBallUp->Set(true);
			upperBallDown->Set(false);
		}
		else
		{
			upperBallDown->Set(true);
			upperBallUp->Set(true);
		}
		
		// Shooter
		
		if (operatorStick->GetRawButton(10))
		{
			shooterMotor->Set(-1.0);
		}
		
		// SAM Jack
		
		if (operatorStick->GetRawButton(6))
		{
			bridgeUp->Set(true);
			bridgeDown->Set(false);
		}
		else
		{
			bridgeUp->Set(false);
			bridgeDown->Set(true);
		}
	}
	
	//
	// AutonomousInit()
	//
	// Called right before entering autonomous mode. Setup the
	// robot for autonomous mode.
	//
	void AutonomousInit()
	{
		autoMode = (int) ds->GetAnalogIn(1);
		autoStep = 1;
		startTime = GetClock();
	}
	
	//
	// AutonomousPeriodic()
	//
	// Called at a regular interval (approx. 20ms) during autonomus mode.
	// Will call one of the autonomous routines below depending on the value of autoMode
	//
	void AutonomousPeriodic()
	{
		switch (autoMode)
		{
		case 0:
			break;
		case 1:
			AutonomousModeOne();	// run Autonomous routine 1
			break;
		default:
			
			break;
		}
	}
	
	void AutonomousModeOne()
	{
		// TODO Finish writing autonomous mode one
		if (autoStep == 1)
		{
			// Power motor and setup pistons
			
			shooterMotor->Set(-1.0);
			upperBallUp->Set(false);
			upperBallDown->Set(true);
			lowerBallUp->Set(true);
			lowerBallDown->Set(false);
			
			// Check if we're ready to advance to the next step
			
			if ((GetClock() - startTime) > 2.0)
			{
				autoStep = 2;
				startTime = GetClock();
			}
		}
		
		else if (autoStep == 2)
		{
			// Fire the first shot
			
			upperBallUp->Set(true);
			upperBallDown->Set(false);
			
			shooterMotor->Set(-1.0);
			
			if ((GetClock() - startTime) > 0.5)
			{
				autoStep++;
				startTime = GetClock();
			}
		}
		
		else if (autoStep == 3)
		{
			
		}
	}
	
	//
	// FindTarget()
	//
	// Analyzes a camera image and updates the position of the target
	//
	void FindTarget()
	{
		AxisCamera &camera = AxisCamera::GetInstance();
		if (camera.IsFreshImage())
		{
			ColorImage image(IMAQ_IMAGE_HSL);
			camera.GetImage(&image);
			
			// HSL Threshold
			
			BinaryImage *binImage;
			binImage = image.ThresholdHSL(128, 255, 13, 255, 249, 255);
			
			// Remove small objects
			
			BinaryImage *bigObjectsImage;
			bigObjectsImage = binImage->RemoveSmallObjects(false, 2);
			
			int size = binImage->GetNumberParticles();
			
			// Convex hull
			
			BinaryImage *convexHullImage;
			convexHullImage = bigObjectsImage->ConvexHull(false);
			
			// Particle analysis
			
			vector <ParticleAnalysisReport> *vPAR = convexHullImage->GetOrderedParticleAnalysisReports();
			
			if (size > 0)
			{
				for (int i = 0; i < (int) vPAR->size(); i++)
				{
					ParticleAnalysisReport *par = &(vPAR->at(i));
					
					int width = par->boundingRect.width;
					int height = par->boundingRect.height;
					float area = par->particleArea;
					
					if (i == 0)
					{
						maxWidth = 0;
						targetWd = 0;
						targetHt = 0;
						targetX = 320;
						targetY = 240;
					}
					
					if ((width > 40) && (area / (width*height)) > 0.75)
					{
						if (width > maxWidth) maxWidth = width;
						if (par->center_mass_y < targetY)
						{
							targetWd = width;
							targetHt = height;
							targetX = par->center_mass_x;
							targetY = par->center_mass_y;
						}
						
						printf("W = %d, H = %d, X = %d, Y = %d\n", targetWd, targetHt, targetX, targetY);
					}
				}
				
				printf("X = %d, Y = %d\n", targetX, targetY);
			}
			
			delete &image;
			delete binImage;
			delete bigObjectsImage;
			delete vPAR;
		} /* camera.IsFreshImage() */
	}
};

START_ROBOT_CLASS(Team316Robot);

