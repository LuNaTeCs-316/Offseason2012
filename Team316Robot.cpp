#include <math.h>
#include "WPILib.h"

/**
 * This is a demo program showing the use of the RobotBase class.
 * The SimpleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 */ 
class Team316Robot : public IterativeRobot
{
private:
	DriverStation *ds;
	
	// Joysticks
	
	Joystick *driverStick;
	Joystick *operatorStick;
	
	// Motors
	
	RobotDrive *driveMotors; 		// robot drive system
	Jaguar *shooterMotor;
	Jaguar *turretMotor;
	
	// Solenoids
	
	Compressor *compressor;
	Solenoid *upperBallUp;
	Solenoid *upperBallDown;
	Solenoid *lowerBallUp;
	Solenoid *lowerBallDown;
	Solenoid *bridgeUp;
	Solenoid *bridgeDown;
	
	// Relays
	
	Relay *ballPickup;
	
	// Counters
	
	Counter *speedCounter;
	
	// Digitial Inputs
	
	DigitalInput *turretLimitLeft;
	DigitalInput *turretLimitRight;
	DigitalInput *ballLoad;
	
	int autoMode;
	int autoStep;
	double startTime;
	
	// Camera data
	
	int maxWidth;
	int targetWd;
	int targetHt;
	int targetX;
	int targetY;
	
public:
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
		
		speedCounter->Start();
		
		AxisCamera &camera = AxisCamera::GetInstance();
		camera.WriteResolution(AxisCamera::kResolution_320x240);
		camera.WriteCompression(40);	// might want to try 10
		camera.WriteBrightness(50);		// might want to try lowering
	}

	void DisabledInit()
	{
		// Put any tasks here to be run when the robot is disabled
	}
	
	void DisablePeriodic() {}	// Nothing to be done here for now
	
	void TeleopInit()
	{
		compressor->Start();
	}
	
	void TeleopPeriodic()
	{
		// Drive Motors
		
		const float DEADBAND = 0.2;
		float drive_x = driverStick->GetX();
		float drive_y = driverStick->GetY();
		float drive_rot = driverStick->GetAxis(Joystick::kTwistAxis);
		if (fabs(drive_x) < DEADBAND) {
			drive_x = 0;
		}
		if (fabs(drive_y) < DEADBAND) {
			drive_y = 0;
		}
		if (fabs(drive_rot) < DEADBAND) {
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
		
		if (turretLimitLeft->Get() && turretVal > 0) {
			turretVal = 0;
		}
		else if (turretLimitRight->Get() && turretVal < 0) {
			turretVal = 0;
		}
		turretMotor->Set(turretVal);
		
		// Ball pickup
		
		if (operatorStick->GetRawButton(2)) {
			ballPickup->Set(Relay::kForward);
		} else {
			ballPickup->Set(Relay::kOff);
		}
		
		// Ball loading/firing
		
		if (operatorStick->GetRawButton(4)) {
			lowerBallDown->Set(true);
			lowerBallUp->Set(false);
		} else if (operatorStick->GetRawButton(5) || ballLoad->Get()) {
			lowerBallUp->Set(true);
			lowerBallDown->Set(false);
		}
		
		if (operatorStick->GetRawButton(1)) {
			upperBallUp->Set(true);
			upperBallDown->Set(false);
		} else {
			upperBallDown->Set(true);
			upperBallUp->Set(true);
		}
		
		// Shooter
		
		if (operatorStick->GetRawButton(10)) {
			shooterMotor->Set(-1.0);
		}
		
		// SAM Jack
		
		if (operatorStick->GetRawButton(6)) {
			bridgeUp->Set(true);
			bridgeDown->Set(false);
		} else {
			bridgeUp->Set(false);
			bridgeDown->Set(true);
		}
	}
	
	void AutonomousInit()
	{
		autoMode = (int) ds->GetAnalogIn(1);
		autoStep = 1;
		startTime = GetClock();
	}
	
	void AutonomousPeriodic()
	{
		switch (autoMode)
		{
		case 0:
			break;
		case 1:
			AutonomousModeOne();
			break;
		default:
			break;
		}
	}
	
	void AutonomousModeOne()
	{
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

