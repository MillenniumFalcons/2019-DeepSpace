package frc.team3647subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import frc.robot.Constants;
import frc.team3647inputs.*;
import frc.robot.Robot;


public class Drivetrain extends Subsystem
{
    public void initDefaultCommand()
	{
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
	}
	
	private WPI_TalonSRX leftSRX = new WPI_TalonSRX(Constants.leftMasterPin);
	private WPI_TalonSRX rightSRX = new WPI_TalonSRX(Constants.rightMasterPin);
	
	private VictorSPX leftSPX1 = new VictorSPX(Constants.leftSlave1Pin);
	private VictorSPX rightSPX1 = new VictorSPX(Constants.rightSlave1Pin);
	private VictorSPX leftSPX2 = new VictorSPX(Constants.leftSlave2Pin);
    private VictorSPX rightSPX2 = new VictorSPX(Constants.rightSlave2Pin);

    private double supposedAngle;

	private DifferentialDrive drive = new DifferentialDrive(leftSRX, rightSRX);

	private int leftEncoderValue = this.getLeftSelectedSensorPosition(Constants.drivePIDIdx); //Left Side of Drivetrain Encoder Position
	private int rightEncoderValue = this.getRightSelectedSensorPosition(Constants.drivePIDIdx); //Right Side of Drivetrain Encoder Position

	private int prevLeftEncoderValue;	//Previous Left Side of Drivetrain Encoder Position
	private int prevRightEncoderValue;	//Previous Right Side of Drivetrain Encoder Position

	//Variables to calculate left and right max velocities
    private double prevLeftVelocity=0, prevRightVelocity=0, maxLeftVelocity=0, maxRightVelocity=0;

    //Variables to store and calculate left and right max acceleration
	private double prevLeftAccel=0, prevRightAccel=0, maxLeftAccel=0, maxRightAccel=0;
	
	Timer maxVelAccelTimer;


    public Drivetrain()
	{
		// Config left side PID settings
		leftSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs);
		leftSRX.setSensorPhase(true);
		leftSRX.configNominalOutputForward(0, Constants.kTimeoutMs);
		leftSRX.configNominalOutputReverse(0, Constants.kTimeoutMs);
		leftSRX.configPeakOutputForward(1, Constants.kTimeoutMs);
		leftSRX.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		// // Config left side PID Values
		// leftSRX.selectProfileSlot(Constants.drivePIDIdx, 0);
		// leftSRX.config_kF(Constants.drivePIDIdx, Constants.lDrivekF, Constants.kTimeoutMs);
		// leftSRX.config_kP(Constants.drivePIDIdx, Constants.lDrivekP, Constants.kTimeoutMs);
		// leftSRX.config_kI(Constants.drivePIDIdx, Constants.lDrivekI, Constants.kTimeoutMs);
		// leftSRX.config_kD(Constants.drivePIDIdx, Constants.lDrivekD, Constants.kTimeoutMs);

		// Config right side PID settings
		rightSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.drivePIDIdx, Constants.kTimeoutMs);
		rightSRX.setSensorPhase(true);
		rightSRX.configNominalOutputForward(0, Constants.kTimeoutMs);
		rightSRX.configNominalOutputReverse(0, Constants.kTimeoutMs);
		rightSRX.configPeakOutputForward(1, Constants.kTimeoutMs);
		rightSRX.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		// // Config right side PID Values
		// rightSRX.selectProfileSlot(Constants.drivePIDIdx, 0);
		// rightSRX.config_kF(Constants.drivePIDIdx, Constants.rDrivekF, Constants.kTimeoutMs);
		// rightSRX.config_kP(Constants.drivePIDIdx, Constants.rDrivekP, Constants.kTimeoutMs);
		// rightSRX.config_kI(Constants.drivePIDIdx, Constants.rDrivekI, Constants.kTimeoutMs);
		// rightSRX.config_kD(Constants.drivePIDIdx, Constants.rDrivekD, Constants.kTimeoutMs);

		// Set up followers
		leftSPX1.follow(leftSRX);
		leftSPX2.follow(leftSRX);

		rightSPX1.follow(rightSRX);
		rightSPX2.follow(rightSRX);
		
		rightSRX.setInverted(true);
		rightSPX1.setInverted(true);
		rightSPX2.setInverted(true);

		leftSRX.configFactoryDefault();
		rightSRX.configFactoryDefault();
    }

	/**
	 * Method to control robot
	 * @param xValue joystick x value
	 * @param yValue joystick y value
	 * @param gyro gyro object
	 */
    public void customArcadeDrive(double xValue, double yValue, Gyro gyro)
	{
		if(yValue != 0 && Math.abs(xValue) < 0.15)
        {
			setPercentOutput(yValue, yValue);
	 	}
		else if(yValue == 0 && Math.abs(xValue) < 0.15)
		{
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
    
    public void setPercentOutput(double lOutput, double rOutput)
	{
		rightSRX.set(ControlMode.PercentOutput, rOutput);
		leftSRX.set(ControlMode.PercentOutput, lOutput);
	}

	
	public void stop()
	{
		rightSRX.stopMotor();
		leftSRX.stopMotor();
    }
	
	//USED BY AUTO FOR SOME REASON
    public void setVelocity(double lSpeed, double rSpeed)
	{
		double targetVelocityRight = rSpeed * Constants.velocityConstant;
		double targetVelocityLeft = lSpeed * Constants.velocityConstant;
		rightSRX.set(ControlMode.Velocity, targetVelocityRight);
		leftSRX.set(ControlMode.Velocity, targetVelocityLeft);
	}

	public void testDrivetrainCurrent()
	{
		System.out.println("Left Motor Current: " + leftSRX.getOutputCurrent());
		System.out.println("Right Motor Current:" + rightSRX.getOutputCurrent());
	}
	
	private void enableCurrentLimiting(double amps)
	{
		leftSRX.enableCurrentLimit(true);
		rightSRX.enableCurrentLimit(true);
	}
	
	public void setToBrake()
	{
		leftSRX.setNeutralMode(NeutralMode.Brake);
		rightSRX.setNeutralMode(NeutralMode.Brake);
		leftSPX1.setNeutralMode(NeutralMode.Brake);
		leftSPX2.setNeutralMode(NeutralMode.Brake);
		rightSPX1.setNeutralMode(NeutralMode.Brake);
		rightSPX2.setNeutralMode(NeutralMode.Brake);
	}
	
	public void setToCoast()
	{
		leftSRX.setNeutralMode(NeutralMode.Coast);
		rightSRX.setNeutralMode(NeutralMode.Coast);
		leftSPX1.setNeutralMode(NeutralMode.Coast);
		leftSPX2.setNeutralMode(NeutralMode.Coast);
		rightSPX1.setNeutralMode(NeutralMode.Coast);
		rightSPX2.setNeutralMode(NeutralMode.Coast);
    }
    
    private void curvatureDrive(double throttle, double turn)
	{
		drive.curvatureDrive(throttle, turn, true);	//curvature drive from WPILIB libraries.
	}

	public int getLeftSelectedSensorPosition(int pidIDx)
	{
		return this.leftSRX.getSelectedSensorPosition(pidIDx);
	}

	public int getRightSelectedSensorPosition(int pidIDx)
	{
		return this.rightSRX.getSelectedSensorPosition(pidIDx);
	}

	public int getLeftSelectedSensorVelocity(int pidIDx)
	{
		return this.leftSRX.getSelectedSensorVelocity(pidIDx);
	}

	public int getRightSelectedSensorVelocity(int pidIDx)
	{
		return this.rightSRX.getSelectedSensorVelocity(pidIDx);
	}

	private void updateEncoders()
	{
		this.prevLeftEncoderValue = this.leftEncoderValue;
		this.prevRightEncoderValue = this.rightEncoderValue;

		leftEncoderValue = this.leftSRX.getSelectedSensorPosition(Constants.drivePIDIdx);
		rightEncoderValue = this.rightSRX.getSelectedSensorPosition(Constants.drivePIDIdx);
	}

	public int getLeftEncoder()
	{
		this.updateEncoders();
		return this.leftEncoderValue;
	}
	public int getRightEncoder()
	{
		this.updateEncoders();
		return this.rightEncoderValue;
	}

	public int getPrevLeftEncoder()
	{
		this.updateEncoders();
		return this.prevLeftEncoderValue;
	}
	public int getPrevRightEncoder()
	{
		this.updateEncoders();
		return this.prevRightEncoderValue;
	}

	public void resetEncoders()
	{
		this.resetPrevLeftEncoder();
		this.resetPrevLeftEncoder();
		this.resetLeftEncoder();
		this.resetRightEncoder();
	}
	
	public void resetLeftEncoder()
	{
		this.leftSRX.setSelectedSensorPosition(0, Constants.drivePIDIdx, Constants.kTimeoutMs);
		this.updateEncoders();
	}

	public void resetRightEncoder()
	{
		this.rightSRX.setSelectedSensorPosition(0, Constants.drivePIDIdx, Constants.kTimeoutMs);
		this.updateEncoders();
	}

	public void resetPrevRightEncoder()
	{
		this.rightEncoderValue = 0;
		this.updateEncoders();
	}

	public void resetPrevLeftEncoder()
	{
		this.leftEncoderValue = 0;
		this.updateEncoders();
	}

	public void configPID()
    {
		// Config left side PID Values
		this.leftSRX.selectProfileSlot(Constants.drivePIDIdx, 0);
		this.leftSRX.config_kF(Constants.drivePIDIdx, Robot.kF, Constants.kTimeoutMs);
		this.leftSRX.config_kP(Constants.drivePIDIdx, Robot.kP, Constants.kTimeoutMs);
		this.leftSRX.config_kI(Constants.drivePIDIdx, Robot.kI, Constants.kTimeoutMs);
		this.leftSRX.config_kD(Constants.drivePIDIdx, Robot.kD, Constants.kTimeoutMs);

		// Config right side PID Values
		this.rightSRX.selectProfileSlot(Constants.drivePIDIdx, 0);
		this.rightSRX.config_kF(Constants.drivePIDIdx, Robot.kF2, Constants.kTimeoutMs);
		this.rightSRX.config_kP(Constants.drivePIDIdx, Robot.kP2, Constants.kTimeoutMs);
		this.rightSRX.config_kI(Constants.drivePIDIdx, Robot.kI2, Constants.kTimeoutMs);
        this.rightSRX.config_kD(Constants.drivePIDIdx, Robot.kD2, Constants.kTimeoutMs);
	}
	
//-- -- -- Methods for testing max velocity and acceleration -- -- -- //
// 	// Resets all values needed for testing max velocity and acceleration
// 	/**
// 	 * This method is required to run before running {@link #VelAccel()} , it resets all the values, the timer object and the encoders
// 	 * 
// 	 */
	public void initializeVelAccel()
	{
		maxVelAccelTimer = new Timer();
		// Resets timer to make the programs run for 2sec
		maxVelAccelTimer.reset();
		maxVelAccelTimer.start();
		this.resetEncoders();

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
	}

	// Puts velocity on the smart dashboard
	public void initializeSmartDashboardVelAccel()
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
	}
	// Converting encoder ticks per 100ms to meters per second (m/s)
	// Takes sensors encoder ticks per .1second (100ms) and wheel radius to get velocity for left and right motors
	private double encoderToMpS(double encoderVal, double wheelRadius)
    {
		// Makes the encoder absolute value in case the motors are backwards or not set inverted
        return Math.abs(encoderVal) / .1  / 4096.0 * .0254 * wheelRadius * Math.PI * 2;
	}

	// Running the motors at full speed in order to test max velocity and acceleration
	private void runMotorsMax()
	{
		this.setPercentOutput(1, 1);
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
	public void velAccel()
	{
		// Runs motors in full power
		this.runMotorsMax();

        // Store current velocity for left motor in meters/second, 3 is wheel radius in inches
		double leftMpS = this.encoderToMpS(this.getLeftSelectedSensorVelocity(0), 3);
		// Store current velocity for right motor in meters/second, 3 is wheel radius in inches
        double rightMpS = this.encoderToMpS(this.getRightSelectedSensorVelocity(0), 3);
        
        // Compare current velocity to max velocity recoreded
        if(leftMpS > this.maxLeftVelocity)
        {
            // If current left velocity is bigger than max, set max to current
            this.maxLeftVelocity = leftMpS;
        }
        if(rightMpS > this.maxRightVelocity)
        {
            // Same for right motor
            this.maxRightVelocity = rightMpS;
        }

		// Initializes the left acceleration and right acceleration variables as 0
		double leftAccel = 0;
		double rightAccel = 0;

		/*
		* In order to ignore the first 20ms of acceleration, the "jolt" at the start and get more accurate acceleration
		* the program checks that the timer is after 20ms and then calculates acceleration
		*/
		if(maxVelAccelTimer.get()>.02)
		{
			// Gets left motor acceleration by subtracting previous velocity from current and divide by .02s (the time between every report for velocity)
			leftAccel = (leftMpS - this.prevLeftVelocity) / .02;
			// Same for right motor
			rightAccel = (rightMpS - this.prevRightVelocity) / .02;

			// Compare previous acceleration to current acceleration
			if(leftAccel > this.maxLeftAccel)
			{
				// If current bigger than max, set max to current
				this.maxLeftAccel = leftAccel;
			}
			if(rightAccel > this.maxRightAccel)
			{
				// If current bigger than max, set max to current
				this.maxRightAccel = rightAccel;
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

        // Sets previous velocity to current velocity before method ends
        this.prevLeftVelocity = leftMpS;
        this.prevRightVelocity = rightMpS;

        // Sets previous acceleraion to current acceleraion before method ends
        this.prevLeftAccel = leftAccel;
        this.prevRightAccel = rightAccel;		
	}

	// Getters for max velocity, left and right
	public double getMaxLeftVelocity()
	{
		return this.maxLeftVelocity;
	}
	public double getMaxRightVelocity()
	{
		return this.maxRightVelocity;
	}

	// Getters for max acceleration, left and right
	public double getMaxLeftAccel()
	{
		return this.maxLeftAccel;
	}

	public double getMaxRightAccel()
	{
		return this.maxRightAccel;
	}

// 	// -- -- -- End of methods to test max velocity and acceleration -- -- -- //
}