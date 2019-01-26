package frc.robot;

public class Constants
{
    //Drive Motor Pins
	public static final int leftMasterPin = 4;		//TO BE CHANGED
	public static final int leftSlave1Pin = 5;		//TO BE CHANGED
	public static final int leftSlave2Pin = 6;		//TO BE CHANGED

	public static final int rightMasterPin = 9;		//TO BE CHANGED
	public static final int rightSlave1Pin = 10;	//TO BE CHANGED
	public static final int rightSlave2Pin = 11;	//TO BE CHANGED


	//Elevator Motor Pins
	public static final int leftGearboxSRX = 1;		//TO BE CHANGED
	public static final int rightGearboxSRX = 1;	//TO BE CHANGED
	public static final int leftGearboxSPX = 1;		//TO BE CHANGED
	public static final int rightGearboxSPX = 1;	//TO BE CHANGED

	//Arm Motor Pins
	public final static int armMasterPin = 0;		//TO BE CHANGED

	//Hatch Intake Motor Pin
	public static final int HatchMotorPin = 3;		//TO BE CHANGED

	//Ball Intake Motor
	public static final int BallMotorPin = 2;

	//Gyro Pin
	public static final int gyroPin = 0;

	//Sensor Pins
	public static final int elevatorBannerSensor = 1;	//TO BE CHANGED
	public static final int armBannerSensor = 1;		//TO BE CHANGED

	//Piston Forward and Reverse Channels
	public static final int BallIntakeFC = 0;
	public static final int HatchIntakeFC = 1;
	
	// AirCompressor Pin
	public static final int CompressorPCMPin = 0; // TO BE CHANGED


	//Timeout Constants
	public final static int kTimeoutMs = 10;	//Universal Constant
	

	//PIDIdx Constsnts
	public final static int drivePIDIdx = 0;
	public final static int armPIDIdx = 0;
	public static final int hatchIntakePIDIdx = 0;

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
	public static int hatchIntakeInside;
	public static int hatchIntakeOutside;
	public static int hatchIntakeLoad;


	//Drivetrain Current Limiting Constants
	public final static int drivePeakCurrent = 35;			//Measured in Amps
	public final static int drivePeakCurrentDuration = 200;	//Measured in Milliseconds
	public final static int driveContinousCurrent = 30;		//Measured in Amps
	

}
