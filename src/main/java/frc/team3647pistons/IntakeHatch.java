package frc.team3647pistons;

import frc.robot.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;

public class IntakeHatch 
{
	public static int currentPosition;
	public static Solenoid piston = new Solenoid(Constants.hatchIntakeFC);
	public static TalonSRX hatchSRX = new TalonSRX(Constants.hatchMotorPin);
	
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
	public static void setPosition(int positionInput)
	{
		if(positionInput == 1)
		{
			setEncPosition(Constants.hatchIntakeInside);
			currentPosition = 1;
		}
		else if(positionInput == 2)
		{
			setEncPosition(Constants.hatchIntakeLoad);
			currentPosition = 2;
		}			
		else if(positionInput == 3)
		{
			setEncPosition(Constants.hatchIntakeOutside);
			currentPosition = 3;
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
