package frc.robot;

public class Constants
{
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~PIN NUMBERS~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
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
	public static final int armSRXPin = 18;	
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
	public static final int elevatorBeamBreakPin = 7;
	public static final int ballShooterBeamBreakPin = 6;

	//Solinoid Pins
	public static final int ballIntakeSolinoidPin = 0;
	public static final int hatchIntakeSolinoidPin = 7;
	public static final int hatchGrabberSolinoidPin = 5;

	//CANifier Pin
	public static final int canifierPin = 19;
	
	// AirCompressor Pin
	public static final int CompressorPCMPin = 0;

	//Timeout Constants
	public static final int kTimeoutMs = 10;	//Universal Constant

	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~PID & MOTION MAGIC VALUES~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	//Arm PID Values
	public static final int armPID = 0;
	public static final double armkP = .75;
	public static final double armkI = 0.00017;
	public static final double armkD = 45;
	public static final double armkF = 0;

	//Arm motion magic
	public static final int kArmSRXCruiseVelocity = 3500;
	public static final int kArmSRXAcceleration = 8000;
	
	//Elevator PID Values
	public static final int interstagePID = 0; //PID Slot Index
	public static final double interstageP = 8.5;
	public static final double interstageI = 0.00004;
	public static final double interstageD = 350;
	public static final double interstageF = 0;
	
	public static final int carriageIdx = 1; //PID Slot Index
	public static final double carriageP = 8;
	public static final double carriageI = 0.00004;
	public static final double carriageD = 80;
	public static final double carriageF = 0;
	
	//Elevator Motion Magic
	public static final int kElevatorCruiseVelocity = 5000;
	public static final int kElevatorAcceleration = 10000;
	
	//Hatch Intake PID Values 
	public static final int kHatchWristPID = 0; //PID Slot Index
	public static final double kHatchWristP = 0.75;
	public static final double kHatchWristI = 0;
	public static final double kHatchWristD = 0.25;
	public static final double kHatchWristF = 0;
	
	//Hatch Intake Motion Magic Values
	public static final int kHatchWristCruiseVelocity = 1750;
	public static final int kHatchWristAcceleration = 2500;
	
	//Drive PID Slot
	public static final int drivePIDIdx = 0;
	public static final int velocityPIDIdx = 0;
	
	//Left Drivetrain PIDF Values
	public static final double lDrivekP = 0.1;
	public static final double lDrivekI = 0;
	public static final double lDrivekD = 0.1;
	public static final double lDrivekF = 0;
	
	//Right Drivetrain PIDF Values
	public static final double rDrivekP = 0.1;
	public static final double rDrivekI = 0;
	public static final double rDrivekD = 0.1;
	public static final double rDrivekF = 0;
	public static final double velocityConstant = 4250; //Explanation for velocityConstant

	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~CURRENT LIMITING VALUES~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	//Drivetrain Current Limiting Constants
	public static final int drivePeakCurrent = 35;			//Measured in Amps
	public static final int drivePeakCurrentDuration = 5000;	//Measured in Milliseconds
	public static final int driveContinousCurrent = 30;		//Measured in Amps

	//Elevator Current Limiting
	public static int kElevatorPeakCurrent = 50;
	public static int kElevatorPeakCurrentDuration = 5000;
	public static int kElevatorContinuousCurrent = 4;

	//Hatch Current Limiting
	public static final int kHatchWristPeakCurrent = 40;
	public static int kHatchWristPeakCurrentDuration = 5000;
	public static final int kHatchWristContinuousCurrent = 5;
	
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~POSITION & THRESHOLD VALUES FOR MECHANISMS~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	//Arm Positions
	//SRX MAG Encoder Positions
	public static final int kArmSRXPositionThreshold = 800; // Threshold for arm positions in encoder values
	public static final int armSRXFlatForwards = 4290;
	public static final int armSRXFlatBackwards = 21650;
	public static final int armSRXCargoL3Front = 24970;
	public static final int armSRXCargoL3Back = 1680;
	public static final int armSRXHatchHandoff = 1700;
	public static final int armSRXStowed = 11000;
	public static final int armSRXVerticalStowed = 12860;
	public static final int armSRXCargoHandoff = 8570;
	public static final int armSRXFwdLimitSwitch = 26150;

	//NEO Hall Effect SEnsor Positions
	public static final double armNEOFlatForwards = 12.666;
	public static final double armNEOFlatBackwards = 63.406;
	public static final double armNEOCargoL3Front = 72.811;
	public static final double armNEOCargoL3Back = 4.976;
	public static final double armNEOHatchHandoff = 5.024;
	public static final double armNEOStowed = 32.714;
	public static final double armNEOVerticalStowed = 37.642;
	public static final double armNEOBallHandoff = 25.071;
	public static final double armNEOFwdLimitSwitch = 77.074;

	//Elevator Positions
	public static final int kElevatorPositionThreshold = 500; // Threshold for arm positions in encoder values
	public static final int elevatorHatchHandoff = 6000;
	public static final int elevatorCargoHandoff = 6000; //need to test this and hatch handoff levels, might be able to combine
	public static final int elevatorCargoShip = 21000;
	public static final int elevatorHatchL2 = 18000;
	public static final int elevatorCargoL2 = 28000;
	public static final int elevatorHatchL3 = 40000;
	public static final int elevatorStowed = 13120; // if lower arm slams ground ball and hatch intakes
	public static final int elevatorVerticalStowed = 14350;
	public static final int elevatorMinRotation = 19000; //Might want to combine with hatch lvl2
	public static final int elevatorStartingStowed = 10700;

	//Hatch Intake Positions
	public static final int kWristPositionThreshold = 25;
	public static final int hatchIntakeGround = 4075;
	public static final int hatchIntakeScore = 1840;
	public static final int hatchIntakeHandoff = 2500;
	

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
