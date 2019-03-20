package frc.team3647subsystems;

import frc.robot.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.*;

public class BallIntake 
{
	private static Solenoid extensionCylinder = new Solenoid(Constants.ballIntakeSolinoidPin);
	private static Solenoid extensionCylinder2 = new Solenoid(Constants.ballIntakeSolinoidPin2);
	private static VictorSPX intakeMotor = new VictorSPX(Constants.ballMotorPin);
	
	public static void ballIntakeinitialization()
	{
		intakeMotor.setInverted(false);
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
		extensionCylinder2.set(true);
	}
	
	public static void retractIntake()
	{
		extensionCylinder.set(false);
		extensionCylinder2.set(false);
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