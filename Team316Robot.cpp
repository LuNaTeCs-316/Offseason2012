#include <math.h>
#include "WPILib.h"

/*****************************************************************************************
 * TODO: write autonomous modes 1
 * TODO: write autonomous modes 2
 * TODO: write autonomous modes 3
 * TODO: write autonomous modes 4
 * TODO: write autonomous modes 5
 * TODO: write - CAMERA_TARGETING
 * TODO: write - DRIVE_TO_BRIDGE
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
 * 		Ball pickup: 
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
const float JOYSTICK_DEADBAND 		= 0.2; 	// total joystick travel is -1.0 TO 1.0
const float DEFAULT_SHOOTER_SPEED 	= -0.9; // in percent 1.0 = 100%
const float FASTEST_SHOOTER_SPEED 	= -1.0; // in percent 1.0 = 100%
const float MEDIUM_FAST_SHOOTER_SPEED 	= -.9; // in percent 1.0 = 100%
const float MEDIUM_SLOW_SHOOTER_SPEED 	= -.75; // in percent 1.0 = 100%
const float SLOWEST_SHOOTER_SPEED 	= -.7; // in percent 1.0 = 100%
const float DEFAULT_TURRET_SPEED 	= .5; 	// in percent 1.0 = 100%

// button desgnations on operator control joystick
const int 	RAISE_FIRING_PISTON_BUTTON  	= 1;
const int 	BALL_PICKUP_BUTTON				= 2;
const int 	CAMERA_TARGETING_BUTTON 		= 3;
const int 	LOWER_LOADING_PISTON_BUTTON 	= 4;
const int 	RAISE_LOADING_PISTON_BUTTON		= 5;
const int 	SAM_JACK_BUTTON					= 6;
const int 	SLOWEST_BALL_SHOOTER_BUTTON		= 7;
const int 	MEDIUM_SLOW_BALL_SHOOTER_BUTTON	= 8;
const int 	MEDIUM_FAST_BALL_SHOOTER_BUTTON	= 9;
const int 	FASTEST_BALL_SHOOTER_BUTTON		= 10;

class Team316Robot : public IterativeRobot
{
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

/***************************************************************************************
 * internal program variables
 ****************************************************************************************/ 
// control of autonomous finite states
int autoMode;		// which autonomous mode are we running?
int autoStep;		// which step in autonomous mode are we in?
double startTime;	// used to record the starting time in autonomous
bool autoTimedOut;	// did autonomous mode timeout
double autoSpeed;	// speed of the shooter in Autonomous Mode

// Camera data
int maxWidth;
int targetWd;
int targetHt;
int targetX;
int targetY;

// Joystick positions for driving robot wheels
float drive_x;
float drive_y;
float drive_y2;
float drive_rot;

// Shooter values
float turretVal; 	

/***************************************************************************************
 * public section is accessible to external classes
 ****************************************************************************************/ 
public:
/***************************************************************************************
 * constructor
 * 
 * here we specify which ports the devices are plugged into
 * 
 * The only constructor; allocates memory for the dynamic member variables.
 ****************************************************************************************/ 
Team316Robot(void) : autoStep(1)
{
	ds = DriverStation::GetInstance();
	
	driverStick = new Joystick(1);
	operatorStick = new Joystick(2);
	
	//	RobotDrive(UINT32 frontLeftMotorChannel, UINT32 rearLeftMotorChannel,
	//				UINT32 frontRightMotorChannel, UINT32 rearRightMotorChannel);
	driveMotors = new RobotDrive(3, 1, 4, 2);
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
} // end of constructor

	
	
/***************************************************************************************
 * Initialize the robot state
 * 
 * this function is called when robot is first enabled to establish our
 * initial operating parameters
 ****************************************************************************************/ 
void RobotInit(void)
{
	// Invert right side drive motors
	driveMotors->SetInvertedMotor(RobotDrive::kFrontRightMotor, true);
	driveMotors->SetInvertedMotor(RobotDrive::kRearRightMotor, true);
	
	driveMotorsControl(0, 0, 0); 	// turn off drive motors
	
	shooterMotor->Set(0);			// turn off shooter motor
	armBall();
	samJackControl(false);			// lower sam jack
	ballPickupControl(false);		// turn off pickup motor

	// Start the counter
	speedCounter->Start();
	
	// Setup the camera
	AxisCamera &camera = AxisCamera::GetInstance();
	camera.WriteResolution(AxisCamera::kResolution_320x240);
	camera.WriteCompression(40);	// might want to try 10
	camera.WriteBrightness(50);		// might want to try lowering

	// Start compressor
	compressor->Start(); // will run in a seperate thread and running depending upon pressure
	
	drive_x = drive_y = drive_rot = 0;
	turretVal = 0;
	
	// test printf
	printf("Robot Initialized...\n");
	
} // end of RobotInit

	
	
/***************************************************************************************
 * Disable the robot
 * 
 * Put any tasks here to be run when the robot is disabled
 * Called once upon entering a disabled state. All outputs should be
 * stopped here for safety.
 ****************************************************************************************/ 
void DisabledInit() {}	// Nothing to be done here for now

	
	
/***************************************************************************************
 * Disable mode
 * 
 * I think this is run inbetween autonomous and teleop modes
 * 
 * Called at a regular interval while the robot is disabled. Place tasks
 *	such as updating/resetting sensor values
 * 
 * TODO: verify when this function is called
 ****************************************************************************************/ 
void DisablePeriodic() {}	// Nothing to be done here for now
	


/***************************************************************************************
 ***************************************************************************************
 *           TELEOP  SECTION
 ***************************************************************************************
 ***************************************************************************************
 ***************************************************************************************
 ***************************************************************************************
 ***************************************************************************************/


/***************************************************************************************
 * Initialize the teleop mode of the robot
 * 
 * this function is called when the robot enters the teleop mode
 * NOTE WELL: this does NOT stop autonomous code from running!  You must do
 * that yourself
 ****************************************************************************************/ 
void TeleopInit()
{
	// restart the compressor as it may have been disabled in autonomous
	// to make shooting more stable
	compressor->Start();
} // end of TeleopInit

	
	
/***************************************************************************************
 * teleop mode 
 * 
 * this function is called every 20ms while the robot is in
 * teleop mode so there is no need to make loops here.
 * 
 * Called at a regular interval (approx. 20ms) during teleop mode.
	The majority of the teleop code goes here. It is important to
	make sure this function is not delayed, so don't place any
	wait statements or loops here.
 * 
 ****************************************************************************************/ 
void TeleopPeriodic()
{
	driveMotorsControl(); 			// control the wheel drive motors using joystick input
	turretControl();				// control the turret if operator joystick moved
	ballPickupControl();			// pickup ball if operator joystick button pressed
	ballLoadingControl();			// operate pistons to position ball in shooter if operator joystick button pressed
	shooterSpeedControl();			// shoot ball if operator joystick button pressed
	samJackControl();				// operate bridge manipulator if operator joystick button pressed
} // end of TeleopPeriodic



/***************************************************************************************
 ***************************************************************************************
 *           AUTONOMOUS  SECTION
 ***************************************************************************************
 ***************************************************************************************
 ***************************************************************************************
 ***************************************************************************************
 ***************************************************************************************/



/***************************************************************************************
 * AutonomousInit
 * 
 * runs when we first enter autonomous mode
 ****************************************************************************************/ 
void AutonomousInit()
{	
	// since ds->GetAnalogIn(1) is a float we cast as an int which should
	// truncate to an integer
	// TODO: check if this rounds up or down or truncates
	autoMode = (int) ds->GetAnalogIn(1);	// automode is predetermined by the analog sliders on the dashboard
	autoStep = 1; 							// start at step 1 not step 0
	startTime = GetClock();					// record the time we start so we can use this number for time based steps
	autoSpeed = (ds->GetAnalogIn(3) * -1) - 5;
} // end of AutonomousInit



/***************************************************************************************
 * AutonomousPeriodic
 * 
 * runs every 20ms during autonomous mode
 * this is a finite state automata built using a case structure
 ****************************************************************************************/ 
void AutonomousPeriodic()
{
	switch (autoMode)	// automode is predetermined by the analog sliders on the dashboard
	{
		case 1:	// Autonomous Mode One
			if (autoStep == 0) autoStep = 1;				
			if (autoStep == 1) INITIAL_SETUP();				// takes 2 seconds by default
			if (autoStep == 2) SHOOT_BALL(); 				// takes 500ms
			if (autoStep == 3) DO_NOTHING_FOR_500MSECONDS();
			if (autoStep == 4) LOAD_NEXT_BALL(); 			// takes 1 second
			if (autoStep == 5)
			{
				if (ballLoad->Get() || GetClock() - startTime > 0.5)
				{
					armBall();
					startTime = GetClock();
					autoStep++;
				}
			}
			if (autoStep == 6) DO_NOTHING_FOR_500MSECONDS();
			if (autoStep == 7) DO_NOTHING_FOR_500MSECONDS();
			if (autoStep == 8) SHOOT_BALL(); 				// takes 500ms
			if (autoStep == 9) PRESET_FOR_TELEOP();
			break;
		case 2:
			if (autoStep == 0) autoStep = 1;				
			if (autoStep == 1) INITIAL_SETUP();				// takes 2 seconds by default
			if (autoStep == 2) SHOOT_BALL(); 				// takes 500ms
			if (autoStep == 3) DO_NOTHING_FOR_500MSECONDS();
			if (autoStep == 4) LOAD_NEXT_BALL(); 			// takes 1 second
			if (autoStep == 5)
			{
				if (ballLoad->Get() || GetClock() - startTime > 0.5)
				{
					armBall();
					startTime = GetClock();
					autoStep++;
				}
			}
			if (autoStep == 6) DO_NOTHING_FOR_500MSECONDS();
			if (autoStep == 7) SHOOT_BALL();	// takes 500ms
			if (autoStep == 8) RAISE_SAM_JACK(); 			
			if (autoStep == 9) DRIVE_TO_BRIDGE_W_DEAD_RECKONING();			
			if (autoStep == 10) LOWER_SAM_JACK();
			if (autoStep == 11) PRESET_FOR_TELEOP();
			break;
		/*case 2:	// Autonomous Mode Two
			if (autoStep == 0) autoStep = 1;				
			if (autoStep == 1) CAMERA_TARGETING(); 			// takes 3 seconds
			if (autoStep == 2) INITIAL_SETUP(1);			// takes 1 seconds
			if (autoStep == 3) SHOOT_BALL(); 				// takes 500ms
			if (autoStep == 4) DO_NOTHING_FOR_500MSECONDS();
			if (autoStep == 5) LOAD_NEXT_BALL(); 			// takes 1 second
			if (autoStep == 6) SHOOT_BALL(); 				// takes 500ms
			if (autoStep == 7) PRESET_FOR_TELEOP();
			break;
		*/
		case 3:
			if (autoStep == 0) autoStep = 1;
			if (autoStep == 1)
			{
				if ((GetClock() - startTime) > ds->GetAnalogIn(2))
				{
					autoStep++;
					startTime = GetClock();
				}
			}
			if (autoStep == 2)
			{
				ballPickup->Set(Relay::kForward);
				if ((GetClock() - startTime) > 2.0)
				{
					ballPickup->Set(Relay::kOff);
					autoStep++;
					startTime = GetClock();
				}	
			}
			if (autoStep == 3) RAISE_SAM_JACK(); 			
			if (autoStep == 4) DRIVE_TO_BRIDGE_W_DEAD_RECKONING();			
			if (autoStep == 5) LOWER_SAM_JACK(); 				
			if (autoStep == 6) PRESET_FOR_TELEOP();
			break;
		case 4:	// Autonomous Mode Four
			if (autoStep == 0) autoStep = 1;				
			if (autoStep == 1) RAISE_SAM_JACK(); 			
			if (autoStep == 2) DRIVE_TO_BRIDGE_W_DEAD_RECKONING();			
			if (autoStep == 3) LOWER_SAM_JACK(); 				
			if (autoStep == 4) PRESET_FOR_TELEOP();
			break;
			
		// insert other modes here	
			
		case 0:				//no autonomous
			break;
		default:
			break;
	} // end of switch
} // end of AutonomousPeriodic



/***************************************************************************************
 * Autonomous mode helper function
 * DO_NOTHING
 ****************************************************************************************/ 
void DO_NOTHING_FOR_500MSECONDS()
{
	if ((GetClock() - startTime) > .5)
	{
		++autoStep;
		startTime = GetClock();
	}
} // end of DO_NOTHING_FOR_500MSECONDS



/***************************************************************************************
 * Autonomous mode helper function
 * PRESET_FOR_TELEOP
 ****************************************************************************************/ 
void PRESET_FOR_TELEOP()
{
	driveMotorsControl(0, 0, 0); 	// turn off drive motors
	shooterMotor->Set(0);			// turn off shooter motor
	reloadBall();
	samJackControl(false);			// lower sam jack
	ballPickupControl(false);		// turn off pickup motor
	// always last step so we do not increment autostep
} // end of PRESET_FOR_TELEOP



/***************************************************************************************
 * Autonomous mode helper function
 * INITIAL_SETUP
 ****************************************************************************************/ 
void INITIAL_SETUP()
{
	// Power motor and setup pistons
	shooterMotor->Set(autoSpeed);
	armBall();
	
	// Check if we're ready to advance to the next step
	if ((GetClock() - startTime) > 3.0)
	{
		++autoStep;
		startTime = GetClock();
	}
} // end of INITIAL_SETUP		


		
void INITIAL_SETUP(int time)
{
	// Power motor and setup pistons
	shooterMotor->Set(autoSpeed);
	armBall();
	// Check if we're ready to advance to the next step
	if ((GetClock() - startTime) > time)
	{
		++autoStep;
		startTime = GetClock();
	}
} // end of INITIAL_SETUP		



void INITIAL_SETUP(float time)
{
// Power motor and setup pistons
	shooterMotor->Set(autoSpeed);
	armBall();
	// Check if we're ready to advance to the next step
	if ((GetClock() - startTime) > time)
	{
		++autoStep;
		startTime = GetClock();
	}
} // end of INITIAL_SETUP		



/***************************************************************************************
 * Autonomous mode helper function
 * SHOOT_BALL
 ****************************************************************************************/ 
void SHOOT_BALL()
{
	shooterMotor->Set(autoSpeed);
	shootBall();
	// Check if we're ready to advance to the next step
	if ((GetClock() - startTime) > 2.0)
	{
		++autoStep;
		startTime = GetClock();
	}
} // end of SHOOT_BALL



/***************************************************************************************
 * Autonomous mode helper function
 * LOAD_NEXT_BALL
 ****************************************************************************************/ 
void LOAD_NEXT_BALL()
{	
	shooterMotor->Set(autoSpeed);
	reloadBall();
	
	// exit after 2 seconds regardless if ball loaded?
	// Check if we're ready to advance to the next step
	if ((GetClock() - startTime) > 2.0)
	{
		++autoStep;
		startTime = GetClock();
	}
} // end of LOAD_NEXT_BALL



/***************************************************************************************
 * Autonomous mode helper function
 * CAMERA_TARGETING
 ****************************************************************************************/ 
void CAMERA_TARGETING()
{
	// grab image
	FindTarget();
	// TODO: write routine that moves turret to target position
} // end of CAMERA_TARGETING 



/***************************************************************************************
 * Autonomous mode helper function
 * DRIVE_TO_BRIDGE_W_GYRO
 ****************************************************************************************/ 
void DRIVE_TO_BRIDGE_W_GYRO()
{
	
} // end of DRIVE_TO_BRIDGE_W_GYRO 



/***************************************************************************************
 * Autonomous mode helper function
 * DRIVE_TO_BRIDGE_W_DEAD_RECKONING
 ****************************************************************************************/ 
void DRIVE_TO_BRIDGE_W_DEAD_RECKONING()
{
	driveMotorsControl(0, 1, 0); 
	if ((GetClock() - startTime) > 1.3)
	{
		++autoStep;
		startTime = GetClock();
	}
} // end of DRIVE_TO_BRIDGE_W_DEAD_RECKONING 



void DRIVE_TO_BRIDGE_W_DEAD_RECKONING(float time)
{
	driveMotorsControl(0, -1, 0); 
	if ((GetClock() - startTime) > time)
	{
		++autoStep;
		startTime = GetClock();
	}
} // end of DRIVE_TO_BRIDGE_W_DEAD_RECKONING 



/***************************************************************************************
 * Autonomous mode helper function
 * LOWER_SAM_JACK
 ****************************************************************************************/ 
void LOWER_SAM_JACK()
{
	samJackControl(false);
	// Check if we're ready to advance to the next step
	if ((GetClock() - startTime) > 2.0)
	{
		++autoStep;
		startTime = GetClock();
	}
} // end of LOWER_SAM_JACK 



/***************************************************************************************
 * Autonomous mode helper function
 * RAISE_SAM_JACK
 ****************************************************************************************/ 
void RAISE_SAM_JACK()
{
	samJackControl(true);
	// Check if we're ready to advance to the next step
	if ((GetClock() - startTime) > 2.0)
	{
		++autoStep;
		startTime = GetClock();
	}
} // end of RAISE_SAM_JACK 




/***************************************************************************************
 ***************************************************************************************
 *           HELPER FUNCTIONS
 ***************************************************************************************
 ***************************************************************************************
 ***************************************************************************************
 ***************************************************************************************
 ***************************************************************************************/



/***************************************************************************************
 * driveMotorsControl 
 * 
 * this function can be called by either autonomous or teleop
 * control functions to move the robot wheels to drive around 
 ****************************************************************************************/ 
void driveMotorsControl()
{
	// Software brake
	if (driverStick->GetRawButton(4))
	{
		printf("Braking...\n");
		static float brakeVal = 0.125;
		drive_x = drive_rot = 0;
		brakeVal *= -1;
		drive_y = brakeVal;
	}
	
	else
	{
		// get current joystick positions
		drive_x = driverStick->GetX();
		drive_y = driverStick->GetY();
		//float y2 = driverStick->GetAxis(Joystick::kDefaultZAxis);
		drive_rot = driverStick->GetAxis(Joystick::kTwistAxis); // y axis of 2nd analog stick
		//float y_final;
		
		// if stick positions are inside of deadband then make them zero
		if (fabs(drive_x) < JOYSTICK_DEADBAND)
			drive_x = 0;
		if (fabs(drive_y) < JOYSTICK_DEADBAND)
			drive_y = 0;
		if (fabs(drive_rot) < JOYSTICK_DEADBAND)
			drive_rot = 0;
	
	// Send data to SmartDashboard
	// SmartDashboard::GetInstance()->PutFloat(drive_y);
	
/*	if (y1 >= 0)
		drive_y = y1 > y2 ? y1 : y2;
	else if (drive_y < 0)
		drive_y = y1 < y2 ? y1 : y2;
	*/
		
	}
	
	// position drive motors according to joystick positions
	driveMotors->MecanumDrive_Cartesian(drive_x, drive_y, drive_rot);
} // end of driveMotorsControl


void driveMotorsControl(int x, int y, int rot)
{
	driveMotors->MecanumDrive_Cartesian(x, y, rot);
} // end of driveMotorsControl


/***************************************************************************************
 * turretControl
 * 
 * this function can be called by either autonomous or teleop
 * control functions to move the shooter turret 
 ****************************************************************************************/ 
void turretControl()
{
	turretVal = operatorStick->GetX();	// read the joystick	
	turretVal *= -DEFAULT_TURRET_SPEED; // reduce speed by xx% and swap motor directions since opposite of joystick

	// if button is pressed then use camera targetting to move turret
	if (operatorStick->GetRawButton(CAMERA_TARGETING_BUTTON))
	{
		/*
		FindTarget();
		error = 160 - targetX;
		turretVal = error * kScaling;
		*/
	}
	
	// if the limit switch is made in the direction we are trying to turn, then stop
	// the motor from trying to turn
	if ((turretLimitLeft->Get() && turretVal > 0) || (turretLimitRight->Get() && turretVal < 0))
		turretVal = 0;
	
	turretMotor->Set(turretVal);
} // end of TurretControl



/***************************************************************************************
 * ballPickup
 * 
 ****************************************************************************************/ 
void ballPickupControl()
{
	if (operatorStick->GetRawButton(BALL_PICKUP_BUTTON)) 
		ballPickup->Set(Relay::kReverse);
	else if (operatorStick->GetRawButton(11))
		ballPickup->Set(Relay::kForward);
	else
		ballPickup->Set(Relay::kOff);
} // end of ballPickup



void ballPickupControl(bool on)
{
	if (on)
		ballPickup->Set(Relay::kForward);
	else
		ballPickup->Set(Relay::kOff);
} // end of ballPickup



/***************************************************************************************
 * ballHandling
 * 
 * Ball loading/firing - operate pistons to position ball in shooter
 ****************************************************************************************/ 
void ballLoadingControl()
{
	// check if the buttons are pressed
	
	static int counter = 0;		// counter is static so that it is only initialized on the first call
		
	if (operatorStick->GetRawButton(RAISE_LOADING_PISTON_BUTTON))
		printf("Button 5 Pressed\n");
	if (operatorStick->GetRawButton(LOWER_LOADING_PISTON_BUTTON))
		printf("Button 4 Pressed\n");
	
	// control of lower piston
	
	if (operatorStick->GetRawButton(LOWER_LOADING_PISTON_BUTTON)) 
	{
		// Lower the loading piston
		
		lowerBallDown->Set(true);
		lowerBallUp->Set(false);
		
		counter = 0;	// set counter to 0 to reset for the next loop
	}
	
	else if ( (operatorStick->GetRawButton(RAISE_LOADING_PISTON_BUTTON) || ballLoad->Get() ) 														|| (ballLoad->Get()) )
	{
		// Raise the loading piston after a delay
		
		counter++;
		
		if (counter >= 20)
		{
			lowerBallUp->Set(true);
			lowerBallDown->Set(false);
		}
			
	}
	
	else
	{
		lowerBallUp->Set(false);
		lowerBallDown->Set(true);
		
		counter = 0;
	}
	
	
	
	// control of upper piston
	if ( (operatorStick->GetRawButton(RAISE_LOADING_PISTON_BUTTON) || ballLoad->Get())
			&& operatorStick->GetRawButton(RAISE_FIRING_PISTON_BUTTON) )
	{
		// raise the firing piston
		
		upperBallUp->Set(true);
		upperBallDown->Set(false);
	}
	
	else
	{
		// lower the firing piston
		
		upperBallDown->Set(true);
		upperBallUp->Set(false);
	}
	
} // end of ballHandling



void reloadBall()
{
	lowerBallDown->Set(true);
	lowerBallUp->Set(false);
	upperBallDown->Set(true);
	upperBallUp->Set(false);
} // end of reloadBall



void armBall()
{
	lowerBallUp->Set(true);
	lowerBallDown->Set(false);
	upperBallDown->Set(true);
	upperBallUp->Set(false);
} // end of armBall



void shootBall()
{
	lowerBallUp->Set(true);
	lowerBallDown->Set(false);
	upperBallUp->Set(true);
	upperBallDown->Set(false);
} // end of shootBall



/***************************************************************************************
 * ballShooter
 * 
 ****************************************************************************************/ 
void shooterSpeedControl()
{
	if ( operatorStick->GetRawButton(FASTEST_BALL_SHOOTER_BUTTON) )
		shooterMotor->Set(DEFAULT_SHOOTER_SPEED);
	else if ( operatorStick->GetRawButton(MEDIUM_FAST_BALL_SHOOTER_BUTTON) )
		shooterMotor->Set(MEDIUM_FAST_SHOOTER_SPEED);
	else if ( operatorStick->GetRawButton(MEDIUM_SLOW_BALL_SHOOTER_BUTTON) )
		shooterMotor->Set(MEDIUM_SLOW_SHOOTER_SPEED);
	else if ( operatorStick->GetRawButton(SLOWEST_BALL_SHOOTER_BUTTON) )
		shooterMotor->Set(SLOWEST_SHOOTER_SPEED);
	else 
		shooterMotor->Set(0);
} // end of ballShooter




/***************************************************************************************
 * samJack
 * 
 ****************************************************************************************/ 
void samJackControl()
{
	if (operatorStick->GetRawButton(SAM_JACK_BUTTON))
	{
		bridgeUp->Set(true);
		bridgeDown->Set(false);
	}
	else
	{
		bridgeUp->Set(false);
		bridgeDown->Set(true);
	}
} // end of samJack



void samJackControl(bool up)
{
	if (up)
	{
		bridgeUp->Set(true);
		bridgeDown->Set(false);
	}
	else
	{
		bridgeUp->Set(false);
		bridgeDown->Set(true);
	}
} // end of samJack



/***************************************************************************************
 * FindTarget
 * 
 * use camera to find basketball backboard
 ****************************************************************************************/ 
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
				} // end of if (i == 0)
				
				if ((width > 40) && (area / (width*height)) > 0.75)
				{
					if (width > maxWidth)
						maxWidth = width;
					
					if (par->center_mass_y < targetY)
					{
						targetWd = width;
						targetHt = height;
						targetX = par->center_mass_x;
						targetY = par->center_mass_y;
					} // end of if (par->center_mass_y < targetY)
					
					printf("W = %d, H = %d, X = %d, Y = %d\n", targetWd, targetHt, targetX, targetY);
				} // end of if ((width > 40) && (area / (width*height)) > 0.75)
			} // end of loop
			
			printf("X = %d, Y = %d\n", targetX, targetY);
		} // end of if (size > 0)
		
		delete &image;
		delete binImage;
		delete bigObjectsImage;
		delete vPAR;
	} // end of  if (camera.IsFreshImage()) 
} // end of findTarget

}; // end of class definition

START_ROBOT_CLASS(Team316Robot);
