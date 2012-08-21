#include <math.h>
#include "WPILib.h"

/*****************************************************************************************
 * TODO List
 * 
 * TODO: verify drive train motors work and are of correct orientation
 * TODO: verify all control buttons and joysticks etc, control the correct devices
 * TODO: write autonomous modes 1, 2, 3, 4, 5
	TODO: write 
		CAMERA_TARGETING
		DRIVE_TO_BRIDGE
		LOWER_SAM_JACK
		RAISE_SAM_JACK
 *
*****************************************************************************************/

/*****************************************************************************************
 * LunatecsOffseason2012
 * class file that will control the lunatecs robot designed to compete in the
 * Rebound Rumble competition.  
 * 
 * Driving: 
 * 		The robot uses 4 motors mecanum wheel drive system that is defined
 * 		with the robotodrive module and controlled by the driverstick which is an xbox 
 * 		controller.  The driver can use either of the 2 analog joysticks on the controller with
 * 		the left joystick moving the robot forward, reverse, or crab left and right.  The right
 * 		joystick moves the robot forward or reverse and rotates left or right.  The driver can also 
 * 		use both joysticks at the same time to yeild a combination of movements.
 * 
 * Shooting:  
 * 		Rebound Rumble is a basketball style game and the lunatecs robot shooting, 
 * 		pickup, etc, is controlled by the operatorstick joystick.  
 * 		
 * 		ball pickup: 
 * 				The robot can pick up balls from the floor using a ball pickup motor 
 * 				triggered by the ballpickup relay.  
 * 		Ball staging: 
 * 				The balls then roll into the shooter staging area of the robot 
 * 				which consists of two pnuematic pistons.  The lower piston prestages the ball 
 * 				so that it is ready to fire.  It is controlled by the lowerballup and lowerballdown 
 * 				solenoids.  The upper piston delivers the ball into the shooter which fires 
 * 				the ball at the basket (hopefully).  The upper piston is controlled by the 
 * 				upperballup and upperballdown solenoids.
 * 		Turret, Shooter, and speed control:
 * 				The shooter consists of a motor controlled by a jaguar PWM speed controller 
 * 				called shootermotor and an optical pickup counter which can see a piece of 
 * 				reflective tape on the side of the shooter motor wheel.  The optical pickup 
 * 				merely counts the number of rotations of the disk and then 
 * 				can calculate the speed of the shooter.  It is called speedcounter here.  
 * 				Finally, the shooter has a turret which can rotate left or right and is 
 * 				controlled by the turretmotor.  To prevent cable damage, the turret is 
 * 				prevented from rotating too far left or right by turret limit switches.  
 * 
 * Balancing: 
 * 		In the end game portion of rebound rumble, robots can balance an a teeter-totter
 * 		style bridge for bonus points.  Our robot uses a single, large, heavy duty piston located
 * 		underneath the robot to raise the entire one side of the robot up over the lip of the bridge
 * 		then lets the weight of the entire robot come down on the bridge to tilt it.  This piston is
 * 		controlled by the bridgeup and bridgedown solenoids.  Once on the bridge, the operator can either
 * 		attempt to balance the robot manually, or push a button to allow the robot to self-balance using
 * 		a gyro sensor.
 * 
 ****************************************************************************************/ 



/****************************************************************************************
 * define global constants 
 ****************************************************************************************/ 
const float JOYSTICK_DEADBAND 			= 0.2; 	//total joystick travel is -1.0 TO 1.0
const float DEFAULT_SHOOTER_SPEED 		= -1.0; //in percent 1.0 = 100%
const float DEFAULT_TURRET_SPEED 		= .5; 	//in percent 1.0 = 100%

//button desgnations on operator control joystick
const int 	RAISE_UPPER_BALL_PISTON_BUTTON  = 1;
const int 	BALL_PICKUP_BUTTON				= 2;
const int 	CAMERA_TARGETING_BUTTON 		= 3;
const int 	LOWER_LOWER_BALL_PISTON_BUTTON 	= 4;
const int 	RAISE__LOWER_BALL_PISTON_BUTTON	= 5;
const int 	SAM_JACK_BUTTON					= 6;
const int 	BALL_SHOOTER_BUTTON				= 10;

class Team316Robot : public IterativeRobot {
	/***************************************************************************************
	 * private section is only accessible internally
	 ****************************************************************************************/ 
private:
	/****************************************************************************************
	 * define pointers to all of the robot parts that are controlled by software 
	 ****************************************************************************************/ 
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
	bool autoTimedOut;	// did autonomous mode timeout
	
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
			// Examine camera image and adjust turret
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
			// Do nothing
			break;
		case 1:
			AutonomousModeOne();	// run Autonomous routine 1
			break;
		default:
			
			break;
		}
	}
	
	//
	// AutonomousModeOne()
	//
	// Fire two shots without aiming
	//
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
			
			if ((GetClock() - startTime) >= 2.0)
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
			
			if ((GetClock() - startTime) >= 0.5)
			{
				autoStep++;
				startTime = GetClock();
			}
		}
		
		else if (autoStep == 3)
		{
			// Wait for the next ball to load
			
			const float TIMEOUT = 1.25;
			autoTimedOut = ((GetClock() - startTime) >= TIMEOUT);
			
			if (ballLoad->Get() || autoTimedOut)
			{
				autoStep++;
				startTime = GetClock();
			}
			
			shooterMotor->Set(-1.0);	// update the shooter motor value so it doesn't time out
		}
		
		else if (autoStep == 4)
		{
			// If ball failed to load, rapidly fire the piston up and down
			// to try and free the ball
			
			if (!autoTimedOut)	// if the ball loaded skip to next step
			{
				autoStep++;
				startTime = GetClock();
			}
			
			else
			{
				// fire the lower piston up and down
				
				shooterMotor->Set(-1.0);	// update the shooter motor value so it doesn't time out
				
				bool state = true;		// state of the piston: up (true) or down (false)
				int stateCount = 0;		// how many times have we switched states?
				
				if (stateCount < 3)		// are we done yet?
				{
					if ((GetClock() - startTime) >= 0.2)	// switch every 200 ms
					{
						state = !state;				// toggle piston state
						startTime = GetClock();		// reset timer
						stateCount++;				// increment state switch count
					}
					
					lowerBallUp->Set(state);		// set the piston state
					lowerBallDown->Set(!state);		// opposite of the other solenoid
				}
				
				else	// 1 second delay
				{
					lowerBallUp->Set(false);		// lower piston so ball can roll in
					lowerBallDown->Set(true);
					
					if ((GetClock() - startTime) >= 1)	// move on after 1 second
					{
						autoStep++;
						startTime = GetClock();
					}
				}
			}
		}		/* autostep == 4 */
		
		else if (autoStep == 5)
		{
			// load the ball and give motor time to reach full speed
			
			lowerBallUp->Set(true);
			lowerBallDown->Set(false);
			
			if ((GetClock() - startTime) >= 2)
			{
				autoStep++;
				startTime = GetClock();
			}
		}
		
		else if (autoStep == 6)
		{
			// fire the second shot
			
			upperBallUp->Set(true);
			upperBallDown->Set(false);
			
			shooterMotor->Set(-1.0);
			
			if ((GetClock() - startTime) >= 0.5)
			{
				autoStep++;
				startTime = GetClock();
			}
		}
		
		else if (autoStep == 7)
		{
			// stop all outputs and prep pistons for teleop
			
			upperBallUp->Set(false);
			upperBallDown->Set(true);
			lowerBallUp->Set(true);
			lowerBallDown->Set(false);
			
			shooterMotor->Set(0.0);
			
			autoStep++;
			
			cout << "AutonomousModeOne finished!" << endl;
		}
		
		else if (autoStep == 8)
		{
			// Nothing to be done; autonomous routine is done finished
		}
	}		/* AutonomousModeOne */
	
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

