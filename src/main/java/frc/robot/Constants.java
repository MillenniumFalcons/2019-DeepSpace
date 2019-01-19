package frc.robot;

public class Constants
{
    //Drive Motor Pins
	public final static int leftMaster = 57;	//TO BE CHANGED
	public final static int rightMaster = 54;	//TO BE CHANGED
	public final static int rightSlave1 = 58;	//TO BE CHANGED
	public final static int rightSlave2 = 59;	//TO BE CHANGED
	public final static int leftSlave1 = 52;	//TO BE CHANGED
	public final static int leftSlave2 = 53;	//TO BE CHANGED

	//Drive PID Constants
	public final static int kTimeoutMs = 10;  //Universal Constant
	
	public final static int drivePID = 0;

	//Arm Constants
	public final static int armMaster = 0;		//TO BE CHANGED
	public final static int armPID = 0;

	
		//Left PIDF Values
	public final static double lDrivekP = 0;
	public final static double lDrivekI = 0;
	public final static double lDrivekD = 0;
	public final static double lDrivekF = 0;

		//Right PIDF Values
	public final static double rDrivekP = 0;
	public final static double rDrivekI = 0;
	public final static double rDrivekD = 0;
	public final static double rDrivekF = 0;
	
	public final static double velocityConstant = 1550; //Explanation for velocityConstant

	//Sensor Pins
	public static final int elevatorBannerSensor = 1;	//TO BE CHANGED

	//Elevator Motor Pins
	public static final int leftGearboxSRX = 1;		//TO BE CHANGED
	public static final int rightGearboxSRX = 1;	//TO BE CHANGED
	public static final int leftGearboxSPX = 1;		//TO BE CHANGED
	public static final int rightGearboxSPX = 1;	//TO BE CHANGED

}