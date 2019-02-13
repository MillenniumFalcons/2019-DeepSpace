package frc.robot;

public class Constants
{
    //Drive Motor Pins
	public static final int leftMasterPin = 9;
	public static final int leftSlave1Pin = 10;	
	public static final int leftSlave2Pin = 11;

	public static final int rightMasterPin = 4;
	public static final int rightSlave1Pin = 5;	
	public static final int rightSlave2Pin = 6;	

	//Elevator Motor Pins
	public static final int ElevatorGearboxSRX = 8;		
	public static final int ElevatorGearboxSPX1 = 12;	
	public static final int ElevatorGearboxSPX2 = 13;		

	//Arm Motor Pins
	public static final int armSRXPin = 18;		//TO BE CHANGED
	public static final int armNEOPin = 17;

	//Hatch Intake Motor Pins
	public static final int hatchMotorPin = 3;		

	//Ball Intake
	public static final int ballMotorPin = 2;

	//Ball Shooter
	public static final int ballShooterPin = 14;

	//Gyro Pin
	public static final int gyroPin = 16;

	//Sensor Pins
	public static final int hatchLimitSwitchPin = 9;
	public static final int hatchIntakeBeamBreakPin = 8;
	public static final int elevatorBreamBreakPin = 7;
	public static final int ballShooterBeamBreakPin = 6;

	//Solinoid Pins
	public static final int ballIntakeSolinoidPin = 0;
	public static final int hatchIntakeSolinoidPin = 7;
	public static final int hatchGrabberSolinoidPin = 5;
	
	// AirCompressor Pin
	public static final int CompressorPCMPin = 0;

	//Timeout Constants
	public static final int kTimeoutMs = 10;	//Universal Constant

	//Hatch Current Limiting
	public static final int kHatchWristPeakCurrent = 40;
	public static final int kHatchWristContinuousCurrentHatch = 5;

	//PID Slot Index
	public static final int drivePIDIdx = 0;
	public static final int armPIDIdx = 0;
	public static final int kHatchWristPID = 0;

	

	//Arm PID Values
	public static final int armIdx = 0;
	public static final int armkP = 0;
	public static final int armkI = 0;
	public static final int armkD = 0;
	public static final int armkF = 0;
	public static final int armIZone = 0;

	//Elevator PID Values
	public static final int interstageIdx = 0;
	public static final double interstageP = 0.3473;
	public static final double interstageI = 0.0005;
	public static final double interstageD = 0.7389;
	public static final double interstageF = 0;

	public static final int carriageIdx = 1;
	public static final double carriageP = 0.1;
	public static final double carriageI = 0;
	public static final double carriageD = 0;
	public static final double carriageF = 0;

	//Elevator Motion Magic
	public static final int kElevatorCruiseVelocity = 8500;
	public static final int kElevatorAcceleration = 29500;

	//Wrist PID Values
	public static final double kHatchWristP = 0.75;
	public static final double kHatchWristI = 0;
	public static final double kHatchWristD = 0.25;
	public static final double kHatchWristF = 0;

	//Wrist Motion Magic Values
	public static final int kHatchWristCruiseVelocity = 1750;
	public static final int kHatchWristAcceleration = 2500;


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

	//Arm Positions
	public static int armFlatForwards;
	public static int armFlatBackwards;
	public static int armCargoL3Front;
	public static int armCargoL3Back;
	public static int armHatchHandoff;
	public static int armStowed;

	// Threshold for arm positions in encoder values
	public static final int armEncoderThreshold = 25;

	//Elevator Positions
	public static final int kElevatorPositionThreshold = 25;
	public static int elevatorrCargoHandoff;
	public static int elevatorHatchHandoff;
	public static int elevatorHatchL2;
	public static final int elevatorHatchL3 = 43000;
	public static int elevatorCargoL2;
	public static int elevatorCargoSHIPL2;
	public static int elevatorStowed;
	public static int armMasterPin;
	public static int elevatorBannerSensor;
	public static double armEncoderStraightForwards;
	public static double armEncoderStraightBackwards;
	public static double armEncoderCargoLevel3Front;
	public static double armEncoderCargoLevel3Back;
	public static double armEncoderHatchHandoff;
	public static double armEncoderHatchIntakeMovement;
	public static double armEncoderRobotStowed;
	public static double armEncoderBallHandoff;
	
	public static final int elevatorLow = 13400;
	public static final int elevatorMiddle = 28000;
	public static final int elevatorHigh = 43000;

	//Hatch Intake Positions
	public static final int kWristPositionThreshold = 25;
	public static final int hatchIntakeGround = 4075;
	public static final int hatchIntakeScore = 1840;

	// Threshold for hatch position in encoder values
	public static final int hatchIntakeEncoderThreshold = 25;

	//Drivetrain Current Limiting Constants
	public static final int drivePeakCurrent = 35;			//Measured in Amps
	public static final int drivePeakCurrentDuration = 200;	//Measured in Milliseconds
	public static final int driveContinousCurrent = 30;		//Measured in Amps
	

	/**************AUTONOMOUS CONSTANTS******************/
	
	public static final double kEncoderTicks = 4096;
	public static final double kWheelDiameter = 6; //inches
	public static final double kWheelBase = 0;
	public static final double kMaxVelocity = 0;

	public static final double kZeta = 0;
	public static final double kBeta = 0;

	public static final double PFTurnkP = 1;
	public static final double PFkA = 0;
	public static final double PFkV = 0;
	public static final double PFkD = 0;
	public static final double PFkI = 0;
	public static final double PFkP = 0;
}
