package frc.team3647pistons;

import frc.robot.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;

public class IntakeHatch 
{
	public static HatchPosition currentPosition;
	public static Solenoid piston = new Solenoid(Constants.hatchIntakeFC);
	public static WPI_TalonSRX hatchSRX = new WPI_TalonSRX(Constants.hatchMotorPin);
	public static DigitalInput limitSwitchIntake = new DigitalInput(Constants.hatchLimitSwitchPin);

	public static void intitialize()
	{
		//Motor Direction
		hatchSRX.set(ControlMode.PercentOutput, 0);
		hatchSRX.setInverted(true);
		hatchSRX.setSensorPhase(true);
		//Current Limiting
		hatchSRX.configPeakCurrentLimit(Constants.peakCurrentHatch, Constants.kTimeoutMs);
		hatchSRX.configContinuousCurrentLimit(Constants.continuousCurrentHatch, Constants.kTimeoutMs);
		hatchSRX.enableCurrentLimit(true);
		//PID
		hatchSRX.config_kP(Constants.hatchIntakePIDIdx, Constants.kPHatch);
		hatchSRX.config_kI(Constants.hatchIntakePIDIdx, Constants.kIHatch);
		hatchSRX.config_kD(Constants.hatchIntakePIDIdx, Constants.kDHatch);
		hatchSRX.config_kF(Constants.hatchIntakePIDIdx, Constants.kFHatch);
		//Positioning
		resetPosition();

		// motion magic
		hatchSRX.configMotionAcceleration(Constants.kHatchAcceleration, Constants.kTimeoutMs);
		hatchSRX.configMotionCruiseVelocity(Constants.kHatchVelocity, Constants.kTimeoutMs);
	}

	public static void configPIDFMM(double p, double i, double d, double f, int vel, int accel)
	{
				//PID
				hatchSRX.config_kP(Constants.hatchIntakePIDIdx, p);
				hatchSRX.config_kI(Constants.hatchIntakePIDIdx, i);
				hatchSRX.config_kD(Constants.hatchIntakePIDIdx, d);
				hatchSRX.config_kF(Constants.hatchIntakePIDIdx, f);
		
				// motion magic
				hatchSRX.configMotionAcceleration(accel, Constants.kTimeoutMs);
				hatchSRX.configMotionCruiseVelocity(vel, Constants.kTimeoutMs);

	}

	public static enum HatchPosition
	{
		INSIDE,
		OUTSIDE,
		LOADING
	}
	
	public static void openIntake()
	{
		piston.set(true);
	}
	
	public static void closeIntake()
	{
		piston.set(false);
	}

	/**
	 * 
	 * @param positionInput 1 = inside drivetrain (0); 2 = loading position for arm (90); 3 = outside intake position (180)
	 */
	public static void setPosition(HatchPosition positionInput)
	{
		if(positionInput == HatchPosition.INSIDE)
		{
			resetPosition();
			//hatchSRX.setSelectedSensorPosition(0,0,Constants.kTimeoutMs);
			currentPosition = HatchPosition.INSIDE;
		}
		else if(positionInput == HatchPosition.LOADING)
		{
			setMMPosition(Constants.hatchIntakeLoad);
			currentPosition = HatchPosition.LOADING;
		}			
		else if(positionInput == HatchPosition.OUTSIDE)
		{
			setMMPosition(Constants.hatchIntakeOutside);
			currentPosition = HatchPosition.OUTSIDE;
		}
		else
			System.out.println("INVALID HATCH INTAKE POSITION INPUT");
	}

	public static void resetPosition()
	{
		while(limitSwitchIntake.get() == true)
		{
			moveMotor(.25);
		}
		hatchSRX.stopMotor();
		hatchSRX.setSelectedSensorPosition(0);
		System.out.println("RESET POSITION");
	}
	
	public static void moveMotor(double power)
	{
		hatchSRX.set(ControlMode.PercentOutput, power);

	}

	public static void setMMPosition(int encoderInput) //MM = Motion Magic
	{
		hatchSRX.set(ControlMode.MotionMagic, encoderInput);
	}
	
	public static void runIntake(boolean joyValue)
	{
		if(joyValue)
		{
			openIntake();
		}
		else
		{
			closeIntake();
		}
	}
}
