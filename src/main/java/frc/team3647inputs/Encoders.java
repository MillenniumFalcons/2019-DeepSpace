package frc.team3647inputs;

import frc.robot.Constants;
import frc.team3647subsystems.*;

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

		leftEncoderValue = Drivetrain.leftSRX.getSelectedSensorPosition(Constants.drivePID);
		rightEncoderValue = Drivetrain.rightSRX.getSelectedSensorPosition(Constants.drivePID);
		armEncoderValue = Arm.armSRX.getSelectedSensorPosition(Constants.armPID);
	}
	
	/**
	 * Used in periodic loop to update encoder position of:
	 * <p> left side of Drivetrain
	 * <p> right side of Drivetrain
	 * <p> arm
	 */
	public void resetEncoders()
	{
		Drivetrain.leftSRX.setSelectedSensorPosition(0, Constants.drivePID, Constants.kTimeoutMs);
		Drivetrain.rightSRX.setSelectedSensorPosition(0, Constants.drivePID, Constants.kTimeoutMs);
		Arm.armSRX.setSelectedSensorPosition(0, Constants.armPID, Constants.kTimeoutMs);
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
	}

	/**
	 * Calculates and prints right and left velocity in encoder units and ft/sec
	 */
	public void printEncoderVelocity()
	{
		rVelocity = Drivetrain.rightSRX.getSelectedSensorVelocity(Constants.drivePID);
		lVelocity = Drivetrain.leftSRX.getSelectedSensorVelocity(Constants.drivePID);
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
		System.out.println("Left Encoder CL Error: " + Drivetrain.leftSRX.getClosedLoopError(Constants.drivePID));
		System.out.println("Right Encoder CL Error: " + Drivetrain.rightSRX.getClosedLoopError(Constants.drivePID));
		System.out.println("Arm Encoder CL Error: " + Arm.armSRX.getClosedLoopError(Constants.armPID));
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
}