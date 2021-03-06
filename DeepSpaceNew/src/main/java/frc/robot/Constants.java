package frc.robot;

public class Constants 
{
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~PIN NUMBERS~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// Drive Motor Pins
	public static final int leftMasterPin = 9;
	public static final int leftSlave1Pin = 12; // 10
	public static final int leftSlave2Pin = 13; // 11

	public static final int rightMasterPin = 4;
	public static final int rightSlave1Pin = 5;
	public static final int rightSlave2Pin = 6;

	// Elevator Motor Pins
	public static final int ElevatorGearboxSRX = 8;
	public static final int ElevatorGearboxSPX1 = 10; // 12
	public static final int ElevatorGearboxSPX2 = 11; // 13
	public static final int ElevatorGearboxSPX3 = 14; // 13

	public static final int ElevatorMasterPDP = 15;
	public static final int ElevatorGearboxSPX1PDP = 13;
	public static final int ElevatorGearboxSPX2PDP = 12;

	// Arm Motor Pins
	public static final int armSRXPin = 18;
	public static final int armNEOPin = 17;

	// Hatch Intake Motor Pins
	public static final int shoppingCartMotorPin = 3;
	public static final int shoppingCartSPXPin = 1; //hatch grabber spx
	public static final int hatchGrabberPDPpin = 7; // 6 pratice, 7 comp

	// Ball Intake
	public static final int ballMotorPin = 2;

	// Ball Shooter
	public static final int ballShooterPin = 15;
	public static final int ballShooterPDPpin = 8; // 9 practice, 8 comp

	// Gyro Pin
	public static final int gyroPin = 16;

	// Sensor Pins
	public static final int hatchLimitSwitchPin = 9;
	// public static final int hatchIntakeBeamBreakPin = 8;
	public static final int elevatorBeamBreakPin = 8;
	public static final int ballShooterBeamBreakPin = 6;

	// Solinoid Pins
	public static final int ballIntakeSolinoidPin = 0;
	public static final int ballIntakeSolinoidPin2 = 3;
	// public static final int hatchIntakeSolinoidPin = 1;
	public static final int hatchGrabberSolinoidPin = 2;
	public static final int mopSolenoidPin = 1;

	// CANifier Pin
	public static final int canifierPin = 0;

	// AirCompressor Pin
	public static final int CompressorPCMPin = 0;

	// Timeout Constants
	public static final int kTimeoutMs = 10; // Universal Constant

	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~PID & MOTION MAGIC VALUES~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	public static final int allSRXPID = 0;
	// Arm PID Values
	public static final int armPID = 0;
	public static final double[] armPIDF = { .8, 0, 30, 0 }; // extended
	public static final double[] armPIDFExtended = { 1, 0, 20, 0 }; // extended

	// public static final double[] armPIDF = { 1, 0, 30, .7 }; good for lvl3

	// Arm motion magic
	public static final int kArmSRXCruiseVelocity = 2200;
	public static final int kArmSRXAcceleration = 4000;

	// Elevator PID Values
	public static final int interstageSlotIdx = 0; // PID Slot Index
	public static final double[] interstagePIDF = { 2.8, 0, 28, 0.105 };

	public static final int carriageSlotIdx = 1; // PID Slot Index
	public static final double[] carriagePIDF = { 11, 0.00004, 80, 0 };

	// Elevator Motion Magic
	public static final int kElevatorCruiseVelocity = 6500; //6500
	public static final int kElevatorAcceleration = 13000; //13000

	// Hatch Intake PID Values
	public static final int kHatchWristPID = 0; // PID Slot Index
	public static final double kHatchWristP = 0.75;
	public static final double kHatchWristI = 0;
	public static final double kHatchWristD = 0.25;
	public static final double kHatchWristF = 0;

	// Hatch Intake Motion Magic Values
	public static final int kHatchWristCruiseVelocity = 1000;
	public static final int kHatchWristAcceleration = 1700;

	// Drive PID Slot
	public static final int driveSlotIdx = 0;
	public static final int velocitySlotIdx = 1;

	// Left and Right Drivetrain Percent PIDF Values (in that order)
	// public static final double[] leftPercentPIDF = { 0.1, 0, 0.1, 0 };
	// public static final double[] rightPercentPIDF = { 0.1, 0, 0.1, 0 };

	// Left and Right Drivetrain Velocity PIDF Values (in that order)
	public static final double[] leftVelocityPIDF = {  .35, 0, 0, .32 }; // .281417 .341 * 1.7, .0005, 3.4
	public static final double[] rightVelocityPIDF = { .39, 0, 0, .31 };

	public static final double[] limelightPID = { .19, 0, 1 }; // p = .2, i = 0, d = 1.5
	public static final double limelightThreshold = .0037;
	public static final double limelightYOffset = -8;
	public static final double limelightAreaThreshold = 9.5;
	public static final double limelightMaxArea = 9.5; //12%
	public static final double limelightMinArea = 0;

	public static final int visionVelocityConstant = 3600;

	public static final String limelightFourbarIP = "http://10.36.47.105:5801/";
	public static final String limelightClimberIP = "http://10.36.47.49:5801/";
	

	public static final double velocityConstant = 3600; // 3600enc / 100ms

	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~CURRENT LIMITING VALUES~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// Drivetrain Current Limiting Constants
	public static final int drivePeakCurrent = 60; // Measured in Amps
	public static final int drivePeakCurrentDuration = 1000; // Measured in Milliseconds
	public static final int driveContinousCurrent = 35; // Measured in Amps

	// Elevator Current Limiting
	public static int kElevatorPeakCurrent = 50;
	public static int kElevatorPeakCurrentDuration = 5000;
	public static int kElevatorContinuousCurrent = 25;
	
	// Hatch Current Limiting
	public static final int kHatchWristPeakCurrent = 40;
	public static int kHatchWristPeakCurrentDuration = 5000;
	public static final int kHatchWristContinuousCurrent = 5;
	
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~POSITION & THRESHOLD VALUES FOR MECHANISMS~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// Arm Positions
	// SRX MAG Encoder Positions
	public static final int kArmSRXPositionThreshold = 400; // Threshold for arm positions in encoder values
	public static final int armSRXFlatForwards = 4500; //4290;
	public static final int armSRXFlatForwardsReset = 4800; //4290;
	public static final int armSRXFlatBackwards = 21900; //22400
	public static final int armSRXFlatBackwardsReset = 22350;
	public static final int armSRXAutoFlatBackwards = 21850; //22400
	public static final int armSRXLevel3FlatForwards = 5200; //4290;
	public static final int armSRXLevel3FlatBackwards = 21500; //22400
	public static final int armSRXCargoL3Front = 7600;
	public static final int armSRXCargoL3Back = 18500;
	public static final int armSRXCargoShipFront = 1300; // ADDED mar 1
	public static final int armSRXCargoShipBack = 24800;
	public static final int armSRXHatchHandoff = 1700;
	public static final int armSRXStowed = 11000;
	public static final int armSRXVerticalStowed = 13070; // alligned to the two bolts
	public static final int armSRXStowedBackwards = 15000;
	public static final int armSRXCargoHandoff = 26000;
	public static final int armSRXFwdLimitSwitch = 26150;
	public static final int armSRXClimb = 20500;
	public static final int armSRXCargoFlatForwards = 4500; 
	public static final int armSRXCargoFlatBackwards = 22200;
	public static final int armSRXResetEncoderVal = 3000;

	public static final int armSRXBackwardLimit = armSRXFlatBackwards + 500;
	public static final int armSRXVerticalLimit = armSRXVerticalStowed - 500;


	// NEO Hall Effect SEnsor Positions
	public static final double armNEOFlatForwards = 12.666;
	public static final double armNEOFlatBackwards = 63.406;
	public static final double armNEOCargoL3Front = 72.811;
	public static final double armNEOCargoL3Back = 4.976;
	public static final double armNEOHatchHandoff = 5.024;
	public static final double armNEOStowed = 32.714;
	public static final double armNEOVerticalStowed = 37.642;
	public static final double armNEOBallHandoff = 25.071;
	public static final double armNEOFwdLimitSwitch = 77.074;

	// Elevator Positions
	public static final int kElevatorPositionThreshold = 500; // Threshold for arm positions in encoder values
	public static final int elevatorCargoL1 = 6200; // need to check
	public static final int elevatorHatchHandoff = 6000;
	public static final int elevatorCargoHandoff = 6150; 
	public static final int elevatorCargoShip = 26000;
	public static final int elevatorHatchL2 = 23000;
	public static final int elevatorCargoL2 = 28400;
	public static final int elevatorCargoL3 = 41000;
	public static final int elevatorHatchL3 = 42800;
	public static final int elevatorStowed = 13120; // if lower arm slams ground ball and hatch intakes
	public static final int elevatorVerticalStowed = 0;
	public static final int elevatorMinRotateL1 = 0;
	public static final int elevatorMinRotation = 5000;
	public static final int elevatorMinRotationFront = 17000; // when going to limit switches, or cargo intake
	public static final int elevatorMinRotationBack = 16000;
	public static final int elevatorStartingStowed = 5000;
	public static final int elevatorCargoLoadingStation = 18300;
	public static final int elevatorBeforeCargoHandoff = 17000;


	// Hatch Intake Positions
	public static final int kWristPositionThreshold = 25;
	public static final int hatchIntakeGround = 4075;
	public static final int hatchIntakeScore = 1840;
	public static final int hatchIntakeHandoff = 2000;

	public static final int shoppingCartDeployed = 2500;

	/************** AUTONOMOUS CONSTANTS ******************/

	public static final double kEncoderTicks = 4096;
	public static final double kWheelDiameter = .153; // meters
	public static final double kWheelBase = .7112;
	public static final double kMaxVelocity = 3; //meter per second

	// 4.22 m/s .153pi meters / second = 1 rev / second

	public static final double kBeta = 2; // b > 0 Correction // 1.6 11/9/19
	public static final double kZeta = .1; // 0 < z < 1 Dampening //.17 hou apr 18th

	public static final double kFieldWidth = 8.2296;
	public static final double expirationTimeSRX = 2; // seconds	
	
	
}
