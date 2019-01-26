package frc.team3647pistons;

import frc.team3647autos.*;
import frc.team3647pistons.*;
import frc.team3647subsystems.*;
import frc.team3647inputs.*;
import frc.robot.*;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class IntakeHatch 
{
	public static DoubleSolenoid piston = new DoubleSolenoid(Constants.HatchInstakeFC, Constants.HatchInstakeRC);
	public static TalonSRX extend = new TalonSRX(Constants.HatchMotorPin);
	
	public static void openIntake()
	{
		piston.set(DoubleSolenoid.Value.kForward);
	}
	
	public static void closeIntake()
	{
		piston.set(DoubleSolenoid.Value.kReverse);
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
