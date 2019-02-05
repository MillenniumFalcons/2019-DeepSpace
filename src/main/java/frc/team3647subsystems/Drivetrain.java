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


public class Drivetrain extends Subsystem
{
    public void initDefaultCommand()
	{
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
	}
	
	public WPI_TalonSRX leftSRX = new WPI_TalonSRX(Constants.leftMasterPin);
	public WPI_TalonSRX rightSRX = new WPI_TalonSRX(Constants.rightMasterPin);
	
	public VictorSPX leftSPX1 = new VictorSPX(Constants.leftSlave1Pin);
	public VictorSPX rightSPX1 = new VictorSPX(Constants.rightSlave1Pin);
	public VictorSPX leftSPX2 = new VictorSPX(Constants.leftSlave2Pin);
    public VictorSPX rightSPX2 = new VictorSPX(Constants.rightSlave2Pin);

    public double supposedAngle;

    public DifferentialDrive drive = new DifferentialDrive(leftSRX, rightSRX);


    public void drivetrainInitialization()
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
	
	public void enableCurrentLimiting(double amps)
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

}