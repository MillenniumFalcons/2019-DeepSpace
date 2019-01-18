package frc.robot;

public class Constants
{
    //Drive Motor Pins
	public final static int leftMaster = 57;
	public final static int rightMaster = 54;
	public final static int rightSlave1 = 58;
	public final static int rightSlave2 = 59;
	public final static int leftSlave1 = 52;
	public final static int leftSlave2 = 53;

	//Drive PID Constants
	public final static int kTimeoutMs = 10;  //Universal Constant
	public final static int drivePID = 0;
	public final static double lDrivekF = 0.65;
	public final static double lDrivekP = 1;//1.75
	public final static double lDrivekI = 0;
	public final static double lDrivekD = 0;
	public final static double rDrivekF = 0.64;
	public final static double rDrivekP = 1;//1.75
	public final static double rDrivekI = 0;
	public final static double rDrivekD = 0;
	public final static double velocityConstant = 1550;

	//Sensor Pins
	public static final int elevatorBannerSensor = 1; //TO BE CHANGED

	//Elevator Motor Pins
	public static final int leftGearboxSRX = 1;
	public static final int rightGearboxSRX = 1;
	public static final int leftGearboxSPX = 1;
	public static final int rightGearboxSPX = 1;

}