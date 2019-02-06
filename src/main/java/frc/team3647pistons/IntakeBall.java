package frc.team3647pistons;

import frc.robot.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.*;

public class IntakeBall 
{
	public static Solenoid piston = new Solenoid(Constants.ballIntakeFC);
	public static VictorSPX ballSRX = new VictorSPX(Constants.ballMotorPin);
	public static DigitalInput reed = new DigitalInput(8);
	
	public static void setSpeed(double speedInput)
	{
		ballSRX.set(ControlMode.PercentOutput, speedInput);
	}

	public static void openIntake()
	{
		piston.set(true);
	}
	
	public static void closeIntake()
	{
		piston.set(false);
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
