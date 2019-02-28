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
				intakeCargo(.75);
				extendIntake();
				BallShooter.intakeCargo(.75);
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
	public static void runSmartIntake(double power)
	{
		if(!BallShooter.cargoDetection())
		{
			intakeCargo(power);
			BallShooter.intakeCargo(power);
		}
		else
		{
			BallShooter.stopMotor();
			stopMotor();
		}
	}
	public static void runIntake()
	{
		if(BallShooter.cargoDetection())
		{
			intakeCargo(.2);
		}
		else
		{
			intakeCargo(.6);
		}
		BallShooter.intakeCargo(1);
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

	public static void intakeCargo(double power)
	{
		setOpenLoop(power);
	}

	public static void backOutIntake()
	{
		setOpenLoop(-1);
	}
}