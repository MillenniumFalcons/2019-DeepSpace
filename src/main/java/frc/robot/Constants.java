package frc.robot;

public class Constants
{
    //Drive Motor Pins
	public final static int leftMasterPin = 57;		//TO BE CHANGED
	public final static int rightMasterPin = 54;	//TO BE CHANGED
	public final static int rightSlave1Pin = 58;	//TO BE CHANGED
	public final static int rightSlave2Pin = 59;	//TO BE CHANGED
	public final static int leftSlave1Pin = 52;		//TO BE CHANGED
	public final static int leftSlave2Pin = 53;		//TO BE CHANGED

	//Elevator Motor Pins
	public static final int leftGearboxSRX = 1;		//TO BE CHANGED
	public static final int rightGearboxSRX = 1;	//TO BE CHANGED
	public static final int leftGearboxSPX = 1;		//TO BE CHANGED
	public static final int rightGearboxSPX = 1;	//TO BE CHANGED

	//Arm Motor Pins
	public final static int armMasterPin = 0;		//TO BE CHANGED

	//Hatch Intake Motor Pin
	public static int HatchMotorPin = 0;			//TO BE CHANGED

	//Sensor Pins
	public static final int elevatorBannerSensor = 1;	//TO BE CHANGED
	public static final int armBannerSensor = 1;		//TO BE CHANGED

	//Piston Forward and Reverse Channels
	public static int BallInstakeFC = 0;
	public static int BallInstakeRC = 0;
	public static int HatchInstakeFC = 0;
	public static int HatchInstakeRC = 0;


	//Timeout Constants
	public final static int kTimeoutMs = 10;	//Universal Constant
	

	//PIDIdx Constsnts
	public final static int drivePIDIdx = 0;
	public final static int armPIDIdx = 0;
	public static int hatchIntakePIDIdx = 0;

	//Arm PID Values
	public final static int armProfile1 = 1;
	public final static int armkP = 0;
	public final static int armkI = 0;
	public final static int armkD = 0;
	public final static int armkF = 0;
	public final static int armIZone = 0;

	//Left Drivetrain PIDF Values
	public final static double lDrivekP = 0;
	public final static double lDrivekI = 0;
	public final static double lDrivekD = 0;
	public final static double lDrivekF = 0;

	//Right Drivetrain PIDF Values
	public final static double rDrivekP = 0;
	public final static double rDrivekI = 0;
	public final static double rDrivekD = 0;
	public final static double rDrivekF = 0;
	public final static double velocityConstant = 1550; //Explanation for velocityConstant

	
	//Arm Positions
	public static int armStraight0;
	public static int armStraight180;
	public static int armHighGoalFront;
	public static int armHighGoalBack;	

	//Elevator Positions
	public static int elevatorLow;
	public static int elevatorMiddle;
	public static int elevatorHigh;


	//Drivetrain Current Limiting Constants
	public final static int drivePeakCurrent = 35;			//Measured in Amps
	public final static int drivePeakCurrentDuration = 200;	//Measured in Milliseconds
	public final static int driveContinousCurrent = 30;		//Measured in Amps
	

}
