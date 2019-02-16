package frc.team3647subsystems;

import frc.robot.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.*;

public class BallIntake 
{
	public static Solenoid extensionCylinder = new Solenoid(Constants.ballIntakeSolinoidPin);
	public static VictorSPX intakeMotor = new VictorSPX(Constants.ballMotorPin);
	
	public static void ballIntakeinitialization()
	{
		intakeMotor.setInverted(false);
	}
	
	public static void runSmartBallIntake(boolean intakeJValue, boolean shootJValue)
	{
		if(intakeJValue)
		{
			if(BallShooter.cargoDetection())
			{
				stopMotor();
				BallShooter.stopMotor();
			}
			else
			{
				intakeCargo();
				extendIntake();
				BallShooter.intakeCargo();
			}
		}
		else if(shootJValue)
		{
			BallShooter.shootBall();
		}
		else
		{
			stopMotor();
			BallShooter.stopMotor();
			retractIntake();
		}
	}
	// For state machine
	public static void runIntake()
	{
		if(!BallShooter.cargoDetection())
		{
			intakeCargo();
			BallShooter.intakeCargo();
		}
	}
	public static void runIntake(boolean joyValue)
	{
		if(joyValue)
		{
			extendIntake();
			intakeCargo();
		}
		else
		{
			stopMotor();
			retractIntake();
		}
	}
	
	public static void setOpenLoop(double speed)
	{
		intakeMotor.set(ControlMode.PercentOutput, speed);
	}

	public static void stopMotor()
	{
		setOpenLoop(0);
	}

	public static void extendIntake()
	{
		extensionCylinder.set(true);
	}
	
	public static void retractIntake()
	{
		extensionCylinder.set(false);
	}

	public static void intakeCargo()
	{
		setOpenLoop(0.5);
	}

	public static void backOutIntake()
	{
		setOpenLoop(-1);
	}
}