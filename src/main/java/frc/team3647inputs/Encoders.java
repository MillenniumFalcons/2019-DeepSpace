package frc.team3647inputs;

import frc.robot.Constants;
import frc.team3647pistons.IntakeHatch;
import frc.team3647subsystems.*;
import frc.robot.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Timer;


public class Encoders 
{
	private int leftEncoderValue;	//Left Side of Drivetrain Encoder Position
	private int rightEncoderValue;	//Right Side of Drivetrain Encoder Position
	private int armEncoderValue;	//Arm Encoder Position
	private int hatchIntakeEncoderValue;
	private int prevLeftEncoderValue;	//Previous Left Side of Drivetrain Encoder Position
	private int prevRightEncoderValue;	//Previous Right Side of Drivetrain Encoder Position
	private int prevArmEncoderValue;	//Previous Arm Encoder Position
	private int prevHatchIntakeEncoderValue;
	private int elevatorEncoderValue;

	


    //Variables to calculate left and right max velocities
    private double prevLeftVelocity, prevRightVelocity, maxLeftVelocity, maxRightVelocity;

    //Variables to store and calculate left and right max acceleration
    private double prevLeftAccel, prevRightAccel, maxLeftAccel, maxRightAccel;
	
	Timer timer;
	
	/**
	 * Used in periodic loop to update previous and current encoder position of:
	 * <p> left side of Drivetrain
	 * <p> right side of Drivetrain
	 * <p> and arm
	 */
	public void updateEncoderValues()
	{
		prevLeftEncoderValue = leftEncoderValue;
		prevRightEncoderValue = rightEncoderValue;
		prevArmEncoderValue = armEncoderValue;
		prevHatchIntakeEncoderValue = hatchIntakeEncoderValue;

		leftEncoderValue = Robot.drivetrain.leftSRX.getSelectedSensorPosition(Constants.drivePIDIdx);
		rightEncoderValue = Robot.drivetrain.rightSRX.getSelectedSensorPosition(Constants.drivePIDIdx);
		armEncoderValue = Robot.arm.armSRX.getSelectedSensorPosition(Constants.armPIDIdx);
		hatchIntakeEncoderValue = IntakeHatch.hatchSRX.getSelectedSensorPosition(Constants.hatchIntakePIDIdx);
		elevatorEncoderValue = Robot.elevator.GearboxMaster.getSelectedSensorPosition(Constants.interstagePIDIdx);
	}
	
	/**
	 * Used in periodic loop to update encoder position of:
	 * <p> left side of Drivetrain
	 * <p> right side of Drivetrain
	 * <p> arm
	 */
	public void resetEncoders()
	{
		Robot.drivetrain.leftSRX.setSelectedSensorPosition(0, Constants.drivePIDIdx, Constants.kTimeoutMs);
		Robot.drivetrain.rightSRX.setSelectedSensorPosition(0, Constants.drivePIDIdx, Constants.kTimeoutMs);
		Robot.arm.armSRX.setSelectedSensorPosition(0, Constants.armPIDIdx, Constants.kTimeoutMs);
		IntakeHatch.hatchSRX.setSelectedSensorPosition(0, Constants.hatchIntakePIDIdx, Constants.kTimeoutMs);
	}

	/**
	 * test encoder by moving robot, use a boolean toggle on joystick to reset encoders
	 * @param joystickToggle joystick boolean toggle button
	 */
	public void testEncodersWithDrive(boolean joystickToggle)
	{
		printEncoderValues();
		if(joystickToggle)
		{
			resetEncoders();
		}
	}
	
	/**
	 * Prints encoder values for:
	 * <p> Left side drivetrain
	 * <p> Right side drivetrain
	 * <p> Arm
	 */
	public void printEncoderValues()
	{
		System.out.println("Left Encoder Value: " + leftEncoderValue);
		System.out.println("Right Encoder Value: " + rightEncoderValue);
		System.out.println("Arm Encoder Value: " + armEncoderValue);
		System.out.println("Hatch Encoder Value: " + hatchIntakeEncoderValue);
	}


	/**
	 * prints closed loop (CL) error of:
	 * <p> left side drivetrain encoder
	 * <p> right side drivetrain encoder.
	 * <p> arm
	 */
 	public void printEncoderCLError()
	{
		System.out.println("Left Encoder CL Error: " + Robot.drivetrain.leftSRX.getClosedLoopError(Constants.drivePIDIdx));
		System.out.println("Right Encoder CL Error: " + Robot.drivetrain.rightSRX.getClosedLoopError(Constants.drivePIDIdx));
		System.out.println("Arm Encoder CL Error: " + Robot.arm.armSRX.getClosedLoopError(Constants.armPIDIdx));
		System.out.println("Hatch Intake Encoder CL Error: " + IntakeHatch.hatchSRX.getClosedLoopError(Constants.hatchIntakePIDIdx));
	}

	/**
	 * @return Left Encoder Value
	 */
	public int getLeftEncoder()
	{
		return leftEncoderValue;
	}

	/**
	 * @return Right Encoder Value
	 */
	public int getRightEncoder()
	{
		return rightEncoderValue;
	}

	/**
	 * @return Arm Encoder Value
	 */
	public int getArmEncoder()
	{
		return armEncoderValue;
	}

	/**
	 * @return Hatch Intake Encoder Value
	 */
	public int getHatchIntakeEncoder()
	{
		return hatchIntakeEncoderValue;
	}

	/**
	 * @return Prev Left Encoder Value
	 */
	public int getPrevLeftEncoder()
	{
		return prevLeftEncoderValue;
	}

	/**
	 * @return Prev Right Encoder Value
	 */
	public int getPrevRightEncoder()
	{
		return prevRightEncoderValue;
	}

	/**
	 * @return Prev Arm Encoder Value
	 */
	public int getPrevArmEncoder()
	{
		return prevArmEncoderValue;
	}

	/**
	 * @return Prev Encoder value for hatch intake floor
	 */
	public int getPrevHatchIntakeEncoder()
	{
		return prevHatchIntakeEncoderValue;
	}

	public int getElevatorEncoder()
	{
		return elevatorEncoderValue;
	}

	//-- -- -- Methods for testing max velocity and acceleration -- -- -- //
	// Resets all values needed for testing max velocity and acceleration
	/**
	 * This method is required to run before running {@link #VelAccel()} , it resets all the values, the timer object and the encoders
	 * 
	 */
	public void initializeVelAccel()
	{
		timer = new Timer();
		// Resets timer to make the programs run for 2sec
		timer.reset();
		timer.start();
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
	private void runMotors()
	{
		// Sets left motor speed max speed
		Robot.drivetrain.leftSRX.set(ControlMode.PercentOutput, 1);
		// Sets right motor speed max speed
        Robot.drivetrain.rightSRX.set(ControlMode.PercentOutput, 1);
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
		this.runMotors();

        // Store current velocity for left motor in meters/second, 3 is wheel radius in inches
		double leftMpS = this.encoderToMpS(Robot.drivetrain.leftSRX.getSelectedSensorVelocity(0), 3);
		// Store current velocity for right motor in meters/second, 3 is wheel radius in inches
        double rightMpS = this.encoderToMpS(Robot.drivetrain.rightSRX.getSelectedSensorVelocity(0), 3);
        
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
		if(timer.get()>.02)
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

	// -- -- -- End of methods to test max velocity and acceleration -- -- -- //

}