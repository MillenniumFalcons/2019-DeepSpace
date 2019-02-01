package frc.team3647inputs;

import frc.robot.Constants;
import frc.team3647pistons.IntakeHatch;
import frc.team3647subsystems.*;
import frc.robot.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.team3647subsystems.Drivetrain;


public class Encoders 
{
	/**
	 * Left Side of Drivetrain Encoder Position
	 */
	private int leftEncoderValue;	//Left Side of Drivetrain Encoder Position
	/**
	 * Right Side of Drivetrain Encoder Position
	 */
	private int rightEncoderValue;	//Right Side of Drivetrain Encoder Position
	/**
	 * Arm Encoder Position
	 */
	private int armEncoderValue;	//Arm Encoder Position
	
	/**
	 * Encoder value for hatch intake floor
	 */
	private int hatchIntakeEncoderValue;

	/**
	 * Previous Left Side of Drivetrain Encoder Position
	 */
	private int prevLeftEncoderValue;	//Previous Left Side of Drivetrain Encoder Position
	/**
	 * Previous Right Side of Drivetrain Encoder Position
	 */
	private int prevRightEncoderValue;	//Previous Right Side of Drivetrain Encoder Position
	/**
	 * Previous Arm Encoder Position
	 */
	private int prevArmEncoderValue;	//Previous Arm Encoder Position
	/**
	 * Encoder value for hatch intake floor
	 */
	private int prevHatchIntakeEncoderValue;

	/**
	 * Right Side Encoder Unit Velocity
	 */
	private int rVelocity;			//Right Side Encoder Unit Velocity
	/**
	 * Left Side Encoder Unit Velocity
	 */
	private int lVelocity;			//Left Side Encoder Unit Velocity
	/**
	 * Right Side velocity in feet per second (FPS)
	 */
	private double rVelocityFPS;	//Right Side velocity in feet per second (FPS)
	/**
	 * Left Side velocity in feet per second (FPS)
	 */
	private double lVelocityFPS;	//Left Side velocity in feet per second (FPS)
	


    //Variables to calculate left and right max velocities
    private double prevLeftVelocity, prevRightVelocity, maxLeftVelocity, maxRightVelocity;

    //Variables to store and calculate left and right max acceleration
    private double prevLeftAccel, prevRightAccel, maxLeftAccel, maxRightAccel;
	
	
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
		armEncoderValue = Arm.armSRX.getSelectedSensorPosition(Constants.armPIDIdx);
		hatchIntakeEncoderValue = IntakeHatch.hatchSRX.getSelectedSensorPosition(Constants.hatchIntakePIDIdx);
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
		Arm.armSRX.setSelectedSensorPosition(0, Constants.armPIDIdx, Constants.kTimeoutMs);
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
	 * Calculates and prints right and left velocity in encoder units and ft/sec
	 */
	public void printEncoderVelocity()
	{
		rVelocity = Robot.drivetrain.rightSRX.getSelectedSensorVelocity(Constants.drivePIDIdx);
		lVelocity = Robot.drivetrain.leftSRX.getSelectedSensorVelocity(Constants.drivePIDIdx);
		rVelocityFPS = (rVelocity / 1440) * 10 * 5 * Math.PI / 12;	//calculate feet per second from right encoder velocity
		lVelocityFPS = (lVelocity / 1440) * 10 * 5 * Math.PI / 12;	//calculate feet per second from left encoder velocity
		System.out.println("Left Encoder Velocity: " + lVelocity + " Left: " + lVelocityFPS + " Ft/Sec");
		System.out.println("Right Encoder Velocity: " + rVelocity + " Right: " + rVelocityFPS + " Ft/Sec");
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
		System.out.println("Arm Encoder CL Error: " + Arm.armSRX.getClosedLoopError(Constants.armPIDIdx));
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

	public void initializeVelAccel()
	{
		this.resetEncoders();
        // Velocity variables initialization
        prevLeftVelocity = 0;
        prevRightVelocity = 0;
        maxLeftVelocity = 0;
        maxRightVelocity = 0;

        //Acceleration variables initialization
        prevLeftAccel = 0;
        prevRightAccel = 0;
        maxLeftAccel = 0;
        maxRightAccel = 0;
	}

	private double encoderToMpS(double encoderVal, double wheelRadius)
    {
        SmartDashboard.putNumber("encoderValue", encoderVal);
        return Math.abs(encoderVal) / .1  / 4096.0 * .0254 * wheelRadius * Math.PI * 2;
	}
	public void runMotors()
	{
		Robot.drivetrain.leftSRX.set(ControlMode.PercentOutput, 1);
        Robot.drivetrain.rightSRX.set(ControlMode.PercentOutput, 1);
	}
	public void velAccel()
	{
        // Takes sensors encoder ticks per .1second and wheel radius to get velocity for left and right motors
        double leftMpS = this.encoderToMpS(Robot.drivetrain.leftSRX.getSelectedSensorVelocity(0), 3);
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

        // Get current acceleration for left and right
        double leftAccel = (leftMpS - this.prevLeftVelocity) / .02;
        double rightAccel = (rightMpS - this.prevRightVelocity) / .02;

        // Compare previous acceleration to current acceleration
        if(leftAccel > this.maxLeftAccel)
        {
            this.maxLeftAccel = leftAccel;
        }
        if(rightAccel > this.maxRightAccel)
        {
            this.maxRightAccel = rightAccel;
        }

        // Displays current and max velocity values on smartdashboard
        // SmartDashboard.putNumber("lMaxVelocity", maxLeftVelocity);
        // SmartDashboard.putNumber("lCurrentVelocity", leftMpS);
        // SmartDashboard.putNumber("rMaxVelocity", maxRightVelocity);
        // SmartDashboard.putNumber("rCurrentVelocity", rightMpS);



        // Displays current and max acceleration values on smartdashboard
        // SmartDashboard.putNumber("lMaxAccel", maxLeftAccel);
        // SmartDashboard.putNumber("lCurrentAccel", leftAccel);
        // SmartDashboard.putNumber("rMaxAccel", maxRightAccel);
        // SmartDashboard.putNumber("rCurrentAccel", rightAccel);

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

}