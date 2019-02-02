package frc.robot;

public class Constants
{
    //Drive Motor Pins
	public static final int leftMasterPin = 9;		//TO BE CHANGED
	public static final int leftSlave1Pin = 10;		//TO BE CHANGED
	public static final int leftSlave2Pin = 11;		//TO BE CHANGED

	public static final int rightMasterPin = 4;		//TO BE CHANGED
	public static final int rightSlave1Pin = 5;	//TO BE CHANGED
	public static final int rightSlave2Pin = 6;	//TO BE CHANGED


	//Elevator Motor Pins
	public static final int leftGearboxSRX = 50;		//TO BE CHANGED
	public static final int rightGearboxSRX = 51;		//TO BE CHANGED
	public static final int leftGearboxSPX = 52;		//TO BE CHANGED
	public static final int rightGearboxSPX = 53;		//TO BE CHANGED

	//Arm Motor Pins
	public static final int armMasterPin = 54;		//TO BE CHANGED

	//Hatch Intake Motor Pins
	public static final int hatchMotorPin = 3;		


	//Ball Intake Motor
	public static final int ballMotorPin = 2;

	//Gyro Pin
	public static final int gyroPin = 16;

	//Sensor Pins
	public static final int elevatorBannerSensor = 1;	//TO BE CHANGED
	public static final int armBannerSensor = 1;		//TO BE CHANGED
	public static final int hatchLimitSwitchPin = 9;

	//Piston Forward and Reverse Channels
	public static final int ballIntakeFC = 0;
	public static final int hatchIntakeFC = 1;
	
	// AirCompressor Pin
	public static final int CompressorPCMPin = 0;	


	//Timeout Constants
	public static final int kTimeoutMs = 10;	//Universal Constant

	//Current Limiting
	public static final int peakCurrentHatch = 40;
	public static final int continuousCurrentHatch = 5;
	

	//PIDIdx Constsnts
	public static final int drivePIDIdx = 0;
	public static final int armPIDIdx = 0;
	public static final int hatchIntakePIDIdx = 0;

	//Arm PID Values
	public static final int armProfile1 = 1;
	public static final int armkP = 0;
	public static final int armkI = 0;
	public static final int armkD = 0;
	public static final int armkF = 0;
	public static final int armIZone = 0;

	//Elevator PID Values
	public static final int interstageIZone = 0;
	public static final double interstageP = 0.1;
	public static final double interstageI = 0;
	public static final double interstageD = 0;
	public static final double interstageF = 0;

	public static final int carriageIZone = 0;
	public static final double carriageP = 0.1;
	public static final double carriageI = 0;
	public static final double carriageD = 0;
	public static final double carriageF = 0;

	//Elevator Motion Magic
	public static final int elevatorCruiseVelocity = 0;
	public static final int elevatorAcceleration = 0;

	//Left Drivetrain PIDF Values
	public static final double lDrivekP = 0.3;
	public static final double lDrivekI = 0;
	public static final double lDrivekD = 0.1;
	public static final double lDrivekF = 0;

	//Right Drivetrain PIDF Values
	public static final double rDrivekP = 0.3;
	public static final double rDrivekI = 0;
	public static final double rDrivekD = 0.1;
	public static final double rDrivekF = 0;
	public static final double velocityConstant = 1550; //Explanation for velocityConstant

	
	/*****************ANY NON-FINAL INTs SHOULD BE FINAL ONCE GIVEN A VALUE*******************/

	//Arm Positions
	public static int armStraight0;
	public static int armStraight180;
	public static int armHighGoalFront;
	public static int armHighGoalBack;	

	//Elevator Positions
	public static int elevatorLow;
	public static int elevatorMiddle;
	public static int elevatorHigh;

	//Hatch Intake Positions
	public static final int hatchIntakeInside = 3800;
	public static final int hatchIntakeLoad = 2150;
	//Motion magic
	public static final int kHatchAcceleration = 5000;
	public static final int kHatchVelocity = 1750;
	//PIDF
	public static final double kPHatch = .65;
	public static final double kIHatch = .00001;
	public static final double kDHatch = .1;
	public static final double kFHatch = 0;

	//PID Slot Index
	public static final int hatchPIDSlot = 0;
	public static final int interstagePIDX = 0;
	public static final int carriagePIDX = 1;


	//Drivetrain Current Limiting Constants
	public static final int drivePeakCurrent = 35;			//Measured in Amps
	public static final int drivePeakCurrentDuration = 200;	//Measured in Milliseconds
	public static final int driveContinousCurrent = 30;		//Measured in Amps
	

	/**************AUTONOMOUS CONSTANTS******************/
	
	public static final double kEncoderTicks = 0;
	public static final double kWheelDiameter = 0;
	public static final double kWheelBase = 0;
	public static final double kMaxVelocity = 0;

	public static final double kZeta = 0;
	public static final double kBeta = 0;




}
