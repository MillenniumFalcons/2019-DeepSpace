package frc.team3647subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import frc.robot.Constants;
import frc.team3647inputs.*;

public class Drivetrain
{
	public static WPI_TalonSRX leftSRX = new WPI_TalonSRX(Constants.leftMasterPin);
	public static WPI_TalonSRX rightSRX = new WPI_TalonSRX(Constants.rightMasterPin);
	
	private static VictorSPX leftSPX1 = new VictorSPX(Constants.leftSlave1Pin);
	private static VictorSPX rightSPX1 = new VictorSPX(Constants.rightSlave1Pin);
	private static VictorSPX leftSPX2 = new VictorSPX(Constants.leftSlave2Pin);
	private static VictorSPX rightSPX2 = new VictorSPX(Constants.rightSlave2Pin);

	
	public static int leftEncoder, rightEncoder;

  public static double supposedAngle;

	private static DifferentialDrive drive = new DifferentialDrive(leftSRX, rightSRX);
	
	public static boolean initialized = false;


  public static void init()
	{
		// Config left side PID settings
		leftSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs);
		leftSRX.setSensorPhase(true);
		leftSRX.configNominalOutputForward(0, Constants.kTimeoutMs);
		leftSRX.configNominalOutputReverse(0, Constants.kTimeoutMs);
		leftSRX.configPeakOutputForward(1, Constants.kTimeoutMs);
		leftSRX.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		// Config right side PID settings
		rightSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.driveSlotIdx, Constants.kTimeoutMs);
		rightSRX.setSensorPhase(true);
		rightSRX.configNominalOutputForward(0, Constants.kTimeoutMs);
		rightSRX.configNominalOutputReverse(0, Constants.kTimeoutMs);
		rightSRX.configPeakOutputForward(1, Constants.kTimeoutMs);
		rightSRX.configPeakOutputReverse(-1, Constants.kTimeoutMs);
		// Set up followers
		leftSPX1.follow(leftSRX);
		leftSPX2.follow(leftSRX);

		rightSPX1.follow(rightSRX);
		rightSPX2.follow(rightSRX);
		
		rightSRX.setInverted(true);
		rightSPX1.setInverted(true);
		rightSPX2.setInverted(true);

		leftSRX.setName("Drivetrain", "leftSRX");
		rightSRX.setName("Drivetrain", "rightSRX");

		// leftSRX.setExpiration(Constants.expirationTimeSRX);
		// rightSRX.setExpiration(Constants.expirationTimeSRX);
		// drive.setExpiration(Constants.expirationTimeSRX);
		drive.setSafetyEnabled(false);
		leftSRX.setSafetyEnabled(false);
		rightSRX.setSafetyEnabled(false);

		
		// resetEncoders();

		setToBrake();

		initialized = true;
	}
	
	public static void selectPIDF(int slot, double[] right, double[] left)
	{
		//PID SLOT
		rightSRX.selectProfileSlot(slot, 0);
		leftSRX.selectProfileSlot(slot, 0);

		//PID
		rightSRX.config_kP(slot, right[0], Constants.kTimeoutMs);		
		rightSRX.config_kI(slot, right[1], Constants.kTimeoutMs);	
		rightSRX.config_kD(slot, right[2], Constants.kTimeoutMs);
		rightSRX.config_kF(slot, right[3], Constants.kTimeoutMs);

		leftSRX.config_kP(slot, left[0], Constants.kTimeoutMs);		
		leftSRX.config_kI(slot, left[1], Constants.kTimeoutMs);	
		leftSRX.config_kD(slot, left[2], Constants.kTimeoutMs);
		leftSRX.config_kF(slot, left[3], Constants.kTimeoutMs);
	}

	/**
	 * Method to control robot
	 * @param xValue joystick x value
	 * @param yValue joystick y value
	 * @param gyro gyro object
	 */
  public static void customArcadeDrive(double xValue, double yValue)
	{
		double threshold = 0.09;
		if(yValue != 0 && Math.abs(xValue) < threshold)
    {
			setPercentOutput(yValue, yValue);
	 	}
		else if(yValue == 0 && Math.abs(xValue) < threshold)
		{
			stop();
		}
		else
		{
			curvatureDrive(xValue, yValue);
		}
	}

	public static void updateEncoders()
	{
		leftEncoder = leftSRX.getSelectedSensorPosition(0);
		rightEncoder = rightSRX.getSelectedSensorPosition(0);
	}
    
  public static void setPercentOutput(double lOutput, double rOutput)
	{
		rightSRX.set(ControlMode.PercentOutput, rOutput);
		leftSRX.set(ControlMode.PercentOutput, lOutput);
	}

	
	public static void stop()
	{
		rightSRX.stopMotor();
		leftSRX.stopMotor();
	}
	
	public static void velocityDrive(double xValue, double yValue, Gyro gyro)
	{
		selectPIDF(Constants.velocitySlotIdx, Constants.rightVelocityPIDF, Constants.leftVelocityPIDF);
		double threshold = 0.09;
		if(yValue != 0 && Math.abs(xValue) < threshold)
        {
			setVelocity(yValue, yValue);
	 	}
		else if(yValue == 0 && Math.abs(xValue) < threshold)
		{
			resetEncoders();
			gyro.resetAngle();
			stop();
			supposedAngle = gyro.getYaw();
		}
		else
		{
			gyro.resetAngle();
			curvatureDrive(xValue, yValue);
			supposedAngle = gyro.getYaw();
		}
	}
	private static void curvatureDrive(double throttle, double turn)
	{
		try
		{
			drive.curvatureDrive(throttle, turn, true);	//curvature drive from WPILIB libraries.
		}
		catch(NullPointerException e)
		{
			System.out.println(e);
			System.out.println("differential drive not initialized\nCreating new DifferentialDrive object");
			drive = new DifferentialDrive(leftSRX, rightSRX);
		}
	}

	//USED BY AUTO FOR SOME REASON
  public static void setVelocity(double lSpeed, double rSpeed)
	{
		double targetVelocityRight = rSpeed * Constants.velocityConstant;
		double targetVelocityLeft = lSpeed * Constants.velocityConstant;
		
		rightSRX.set(ControlMode.Velocity, targetVelocityRight);
		leftSRX.set(ControlMode.Velocity, targetVelocityLeft);
	}

	public static void setAutoVelocity(double leftDriveSignal, double rightDriveSignal)
	{
		rightSRX.set(ControlMode.Velocity, rightDriveSignal);
		leftSRX.set(ControlMode.Velocity, leftDriveSignal);
	}

	public static void testDrivetrainCurrent()
	{
		System.out.println("Left Motor Current: " + leftSRX.getOutputCurrent());
		System.out.println("Right Motor Current:" + rightSRX.getOutputCurrent());
	}

	public static void printEncoders()
	{
		System.out.println("Left Encoder: " + leftSRX.getSelectedSensorPosition());
		System.out.println("Right Encoder:" + rightSRX.getSelectedSensorPosition());
	}

	public static void printVelocity()
	{
		System.out.println("Left Vel: " + leftSRX.getSelectedSensorVelocity());
		System.out.println("Right Vel:" + rightSRX.getSelectedSensorVelocity());
	}

	public static int prevVelR = 0, prevVelL = 0;
	public static void printAccel()
	{
		int currentVelR = rightSRX.getSelectedSensorVelocity();
		int currentVelL = leftSRX.getSelectedSensorVelocity();

		System.out.println("Left accel: " + (currentVelL - prevVelL) / .02);
		System.out.println("Right Vel:" + (currentVelR - prevVelR) / .02);

		prevVelR = rightSRX.getSelectedSensorVelocity();
		prevVelL = leftSRX.getSelectedSensorVelocity();
	}

	public static void printVelError()
	{
		int velErrorR = rightSRX.getClosedLoopError();
		int velErrorL = leftSRX.getClosedLoopError();
		System.out.println("Right Vel Error: " + velErrorR);
		System.out.println("Left Vel Error: " + velErrorL);
	}
	public static void enableCurrentLimiting(double amps)
	{
		leftSRX.enableCurrentLimit(true);
		rightSRX.enableCurrentLimit(true);
	}
	
	public static void setToBrake()
	{
		leftSRX.setNeutralMode(NeutralMode.Brake);
		rightSRX.setNeutralMode(NeutralMode.Brake);
		leftSPX1.setNeutralMode(NeutralMode.Brake);
		leftSPX2.setNeutralMode(NeutralMode.Brake);
		rightSPX1.setNeutralMode(NeutralMode.Brake);
		rightSPX2.setNeutralMode(NeutralMode.Brake);
	}
	
	public static void setToCoast()
	{
		leftSRX.setNeutralMode(NeutralMode.Coast);
		rightSRX.setNeutralMode(NeutralMode.Coast);
		leftSPX1.setNeutralMode(NeutralMode.Coast);
		leftSPX2.setNeutralMode(NeutralMode.Coast);
		rightSPX1.setNeutralMode(NeutralMode.Coast);
		rightSPX2.setNeutralMode(NeutralMode.Coast);
    }
    


	public static void resetEncoders()
	{
		leftSRX.setSelectedSensorPosition(0);
		rightSRX.setSelectedSensorPosition(0);
	}

	public static void setEncoders(int leftVal, int rightVal)
	{
		leftSRX.setSelectedSensorPosition(leftVal);
		rightSRX.setSelectedSensorPosition(rightVal);
	}

		//-- -- -- Methods for testing max velocity and acceleration -- -- -- //
	// 	// Resets all values needed for testing max velocity and acceleration
	// 	/**
	// 	 * This method is required to run before running {@link #VelAccel()} , it resets all the values, the timer object and the encoders
	// 	 * 
	// 	 */
	private static double prevLeftVelocity, prevRightVelocity;
	private static double maxLeftVelocity, maxRightVelocity;

	private static double prevLeftAccel, prevRightAccel;
	private static double maxLeftAccel, maxRightAccel;

	// private static double prevRightJerk, prevLeftJerk;
	private static double maxRightJerk, maxLeftJerk;
	private static Timer maxVelAccelTimer;
	public static void initializeVelAccel()
	{
		maxVelAccelTimer = new Timer();
		// Resets timer to make the programs run for 2sec
		maxVelAccelTimer.reset();
		maxVelAccelTimer.start();
		resetEncoders();

		// Velocity variables initialization (and reset)
		//Used to get the acceleration of left Motor with the current velocity. (derivative)
		prevLeftVelocity = 0;
		//Used to get the acceleration of right Motor with the current velocity. (derivative)
		prevRightVelocity = 0;
		
		// Will store the max retrieved velocity at every point while the velAccel() method runs
		maxLeftVelocity = 0;
		maxRightVelocity = 0;

		//Acceleration variables initialization (and reset)
		// prev Accel isn't currently used
		prevLeftAccel = 0;
		prevRightAccel = 0;
		
		// Will store the max calculated acceleration at every point while the velAccel() method runs
		maxLeftAccel = 0;
		maxRightAccel = 0;

		// prevLeftJerk = 0;
		// prevRightJerk = 0;

		maxRightJerk = 0;
		maxLeftJerk = 0;
	}

	// Puts velocity on the smart dashboard
	public static void initializeSmartDashboardVelAccel()
	{
		SmartDashboard.putNumber("lMaxVelocity", 0);
		SmartDashboard.putNumber("lCurrentVelocity", 0);
		SmartDashboard.putNumber("rMaxVelocity", 0);
		SmartDashboard.putNumber("rCurrentVelocity", 0);


		SmartDashboard.putNumber("lMaxAccel", 0);
		SmartDashboard.putNumber("lCurrentAccel", 0);
		SmartDashboard.putNumber("rMaxAccel", 0);
		SmartDashboard.putNumber("rCurrentAccel", 0);
		SmartDashboard.putNumber("encoderValue", 0);

		// Displays current and max acceleration values on smartdashboard
		SmartDashboard.putNumber("lMaxJerk", 0);
		SmartDashboard.putNumber("lCurrentJerk", 0);
		SmartDashboard.putNumber("rMaxJerk", 0);
		SmartDashboard.putNumber("rCurrentJerk", 0);
	}
	// Converting encoder ticks per 100ms to meters per second (m/s)
	// Takes sensors encoder ticks per .1second (100ms) and wheel radius to get velocity for left and right motors
	private static double encoderToMpS(double encoderVal, double wheelRadius)
	{
		// Makes the encoder absolute value in case the motors are backwards or not set inverted
		return Math.abs(encoderVal) / .1  / 4096.0 * .0254 * wheelRadius * Math.PI * 2;
	}

	// Running the motors at full speed in order to test max velocity and acceleration
	private static void runMotorsMax()
	{
		leftSRX.set(ControlMode.Velocity, 3250);
		rightSRX.set(ControlMode.Velocity, 3250);
	}
	/**
	 * A method that calculates max acceleration and velocity for robot, requires running {@link #initializeVelAccel()} , as well
	 * <p>
	 * The method uses getSelectedSensorVelocity(0) an SRX method that gives encoder ticks / 100ms, it then converts to 
	 * meters/second and uses conditional to determine max velocity.
	 * </p>
	 * 
	 * <p>
	 * To get max acceleration the method subtracts previous velocity from current velocity and divides by .02s, the time 
	 * between every report of velocity from the motor controller. It skips the first 20ms when calculating max acceleration
	 * in order to ignore the first "jolt" of the robot and get more accurate values.
	 * </p>
	 * 
	 */
	public static void velAccel()
	{
		// Runs motors in full power
		runMotorsMax();

		// Store current velocity for left motor in meters/second, 3 is wheel radius in inches
		double leftMpS = encoderToMpS(leftSRX.getSelectedSensorVelocity(0), 3);
		// Store current velocity for right motor in meters/second, 3 is wheel radius in inches
		double rightMpS = encoderToMpS(rightSRX.getSelectedSensorVelocity(0), 3);
				
		// Compare current velocity to max velocity recoreded
		if(leftMpS > maxLeftVelocity)
		{
				// If current left velocity is bigger than max, set max to current
				maxLeftVelocity = leftMpS;
		}
		if(rightMpS > maxRightVelocity)
		{
				// Same for right motor
				maxRightVelocity = rightMpS;
		}

		// Initializes the left acceleration and right acceleration variables as 0
		double leftAccel = 0;
		double rightAccel = 0;

		double rightJerk = 0;
		double leftJerk = 0;

		/*
		* In order to ignore the first 20ms of acceleration, the "jolt" at the start and get more accurate acceleration
		* the program checks that the timer is after 20ms and then calculates acceleration
		*/
		if(maxVelAccelTimer.get()>.02)
		{
			// Gets left motor acceleration by subtracting previous velocity from current and divide by .02s (the time between every report for velocity)
			leftAccel = (leftMpS - prevLeftVelocity) / .02;
			// Same for right motor
			rightAccel = (rightMpS - prevRightVelocity) / .02;

			leftJerk = (leftAccel - prevLeftAccel) / .02;
			rightJerk = (rightAccel - prevRightAccel) / .02;
			// Compare previous acceleration to current acceleration
			if(leftAccel > maxLeftAccel)
			{
				// If current bigger than max, set max to current
				maxLeftAccel = leftAccel;
			}
			if(rightAccel > maxRightAccel)
			{
				// If current bigger than max, set max to current
				maxRightAccel = rightAccel;
			}

			if(leftJerk > maxLeftJerk)
			{
				maxLeftJerk = leftJerk;
			}
			if(rightJerk > maxRightJerk)
			{
				maxRightJerk = rightJerk;
			}
			
		}
				// Displays current and max velocity values on smartdashboard
				SmartDashboard.putNumber("lMaxVelocity", maxLeftVelocity);
				SmartDashboard.putNumber("lCurrentVelocity", leftMpS);
				SmartDashboard.putNumber("rMaxVelocity", maxRightVelocity);
				SmartDashboard.putNumber("rCurrentVelocity", rightMpS);



				// Displays current and max acceleration values on smartdashboard
				SmartDashboard.putNumber("lMaxAccel", maxLeftAccel);
				SmartDashboard.putNumber("lCurrentAccel", leftAccel);
				SmartDashboard.putNumber("rMaxAccel", maxRightAccel);
				SmartDashboard.putNumber("rCurrentAccel", rightAccel);

				// Displays current and max acceleration values on smartdashboard
				SmartDashboard.putNumber("lMaxJerk", maxLeftJerk);
				SmartDashboard.putNumber("lCurrentJerk", leftJerk);
				SmartDashboard.putNumber("rMaxJerk", maxRightJerk);
				SmartDashboard.putNumber("rCurrentJerk", rightJerk);

				// Sets previous velocity to current velocity before method ends
				prevLeftVelocity = leftMpS;
				prevRightVelocity = rightMpS;

				// Sets previous acceleraion to current acceleraion before method ends
				prevLeftAccel = leftAccel;
				prevRightAccel = rightAccel;		
	}

	// Getters for max velocity, left and right
	public static double getMaxLeftVelocity()
	{
		return maxLeftVelocity;
	}
	public static double getMaxRightVelocity()
	{
		return maxRightVelocity;
	}

	// Getters for max acceleration, left and right
	public static double getMaxLeftAccel()
	{
		return maxLeftAccel;
	}

	public static double getMaxRightAccel()
	{
		return maxRightAccel;
	}

	// 	// -- -- -- End of methods to test max velocity and acceleration -- -- -- //

}
