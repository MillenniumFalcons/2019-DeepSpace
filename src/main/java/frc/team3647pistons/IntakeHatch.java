package frc.team3647pistons;

import frc.robot.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;

public class IntakeHatch 
{
	public static HatchPosition currentPosition;
	public static Solenoid piston = new Solenoid(Constants.hatchIntakeFC);
	public static TalonSRX hatchSRX = new TalonSRX(Constants.hatchMotorPin);

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
			setEncPosition(Constants.hatchIntakeOutside);
			currentPosition = HatchPosition.OUTSIDE;
		}
		else
			System.out.println("INVALID HATCH INTAKE POSITION INPUT");
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
