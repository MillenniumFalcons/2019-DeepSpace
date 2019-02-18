package frc.team3647subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import frc.robot.Constants;
import frc.team3647inputs.*;


public class Drivetrain
{
	
	public static WPI_TalonSRX leftSRX = new WPI_TalonSRX(Constants.leftMasterPin);
	public static WPI_TalonSRX rightSRX = new WPI_TalonSRX(Constants.rightMasterPin);
	
	public static VictorSPX leftSPX1 = new VictorSPX(Constants.leftSlave1Pin);
	public static VictorSPX rightSPX1 = new VictorSPX(Constants.rightSlave1Pin);
	public static VictorSPX leftSPX2 = new VictorSPX(Constants.leftSlave2Pin);
    public static VictorSPX rightSPX2 = new VictorSPX(Constants.rightSlave2Pin);

    public static double supposedAngle;

    public static DifferentialDrive drive = new DifferentialDrive(leftSRX, rightSRX);


    public static void drivetrainInitialization()
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
		// leftSRX.config_kP(Constants.drivePIDIdx, Constants.leftPercentPIDF[0], Constants.kTimeoutMs);
		// leftSRX.config_kI(Constants.drivePIDIdx, Constants.leftPercentPIDF[1], Constants.kTimeoutMs);
		// leftSRX.config_kD(Constants.drivePIDIdx, Constants.leftPercentPIDF[2], Constants.kTimeoutMs);
		// leftSRX.config_kF(Constants.drivePIDIdx, Constants.leftPercentPIDF[3], Constants.kTimeoutMs);

		// Config right side PID settings
		rightSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.driveSlotIdx, Constants.kTimeoutMs);
		rightSRX.setSensorPhase(true);
		rightSRX.configNominalOutputForward(0, Constants.kTimeoutMs);
		rightSRX.configNominalOutputReverse(0, Constants.kTimeoutMs);
		rightSRX.configPeakOutputForward(1, Constants.kTimeoutMs);
		rightSRX.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		// Config right side PID Values
		// rightSRX.selectProfileSlot(Constants.drivePIDIdx, 0);
		// rightSRX.config_kP(Constants.drivePIDIdx, Constants.rightPercentPIDF[0], Constants.kTimeoutMs);
		// rightSRX.config_kI(Constants.drivePIDIdx, Constants.rightPercentPIDF[1], Constants.kTimeoutMs);
		// rightSRX.config_kD(Constants.drivePIDIdx, Constants.rightPercentPIDF[2], Constants.kTimeoutMs);
		// rightSRX.config_kF(Constants.drivePIDIdx, Constants.rightPercentPIDF[3], Constants.kTimeoutMs);

		// Set up followers
		leftSPX1.follow(leftSRX);
		leftSPX2.follow(leftSRX);

		rightSPX1.follow(rightSRX);
		rightSPX2.follow(rightSRX);
		
		rightSRX.setInverted(true);
		rightSPX1.setInverted(true);
		rightSPX2.setInverted(true);

		setToBrake();
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
		

		//Motion Magic Constants
		// rightSRX.configMotionCruiseVelocity(vel, Constants.kTimeoutMs);
		// rightSRX.configMotionAcceleration(accel, Constants.kTimeoutMs);
		
		// leftSRX.configMotionCruiseVelocity(vel, Constants.kTimeoutMs);
        // leftSRX.configMotionAcceleration(accel, Constants.kTimeoutMs);
	}

	/**
	 * Method to control robot
	 * @param xValue joystick x value
	 * @param yValue joystick y value
	 * @param gyro gyro object
	 */
    public static void customArcadeDrive(double xValue, double yValue, Gyro gyro)
	{
		double threshold = 0.09;
		if(yValue != 0 && Math.abs(xValue) < threshold)
        {
			setPercentOutput(yValue, yValue);
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
	
	//USED BY AUTO FOR SOME REASON
    public static void setVelocity(double lSpeed, double rSpeed)
	{
		double targetVelocityRight = rSpeed * Constants.velocityConstant;
		double targetVelocityLeft = lSpeed * Constants.velocityConstant;
		rightSRX.set(ControlMode.Velocity, targetVelocityRight);
		leftSRX.set(ControlMode.Velocity, targetVelocityLeft);
	}

	public static void testDrivetrainCurrent()
	{
		System.out.println("Left Motor Current: " + leftSRX.getOutputCurrent());
		System.out.println("Right Motor Current:" + rightSRX.getOutputCurrent());
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
    
    private static void curvatureDrive(double throttle, double turn)
	{
		drive.curvatureDrive(throttle, turn, true);	//curvature drive from WPILIB libraries.
	}

	public static void resetEncoders()
	{
		leftSRX.setSelectedSensorPosition(0);
		rightSRX.setSelectedSensorPosition(0);
	}

}