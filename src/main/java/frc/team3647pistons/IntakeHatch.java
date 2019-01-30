package frc.team3647pistons;

import frc.robot.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;

public class IntakeHatch 
{
	public static HatchPosition currentPosition;
	public static Solenoid piston = new Solenoid(Constants.hatchIntakeFC);
	public static TalonSRX hatchSRX = new TalonSRX(Constants.hatchMotorPin);
	public static DigitalInput limitSwitchIntake = new DigitalInput(Constants.hatchLimitSwitchPin);

	public static void intitialize()
	{
		//Motor Direction
		hatchSRX.setInverted(true);
		hatchSRX.setSensorPhase(true);
		//Current Limiting
		hatchSRX.configPeakCurrentLimit(Constants.peakCurrentHatch, Constants.kTimeoutMs);
		hatchSRX.configContinuousCurrentLimit(Constants.continuousCurrentHatch, Constants.kTimeoutMs);
		hatchSRX.enableCurrentLimit(true);
		//PID
		hatchSRX.config_kP(Constants.hatchPIDSlot, Constants.kPHatch);
		hatchSRX.config_kI(Constants.hatchPIDSlot, Constants.kIHatch);
		hatchSRX.config_kD(Constants.hatchPIDSlot, Constants.kDHatch);
		hatchSRX.config_kF(Constants.hatchPIDSlot, Constants.kFHatch);
		//Positioning
		resetPosition();

		// motion magic
		hatchSRX.configMotionAcceleration(Constants.kHatchAcceleration, Constants.kTimeoutMs);
		hatchSRX.configMotionCruiseVelocity(Constants.kHatchVelocity, Constants.kTimeoutMs);
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
			setEncPosition(Constants.hatchIntakeInside);
			currentPosition = HatchPosition.INSIDE;
		}
		else if(positionInput == HatchPosition.LOADING)
		{
			setEncPosition(Constants.hatchIntakeLoad);
			currentPosition = HatchPosition.LOADING;
		}			
		else if(positionInput == HatchPosition.OUTSIDE)
		{
			resetPosition();
			currentPosition = HatchPosition.OUTSIDE;
		}
		else
			System.out.println("INVALID HATCH INTAKE POSITION INPUT");
	}

	public static void resetPosition()
	{
		if(limitSwitchIntake.get())
		{
			moveMotor(-.35);
		}
		else
		{
			moveMotor(0);
			hatchSRX.setSelectedSensorPosition(0);
		}
	}
	
	public static void moveMotor(double power)
	{
		hatchSRX.set(ControlMode.PercentOutput, power);

	}

	public static void setEncPosition(int encoderInput)
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
