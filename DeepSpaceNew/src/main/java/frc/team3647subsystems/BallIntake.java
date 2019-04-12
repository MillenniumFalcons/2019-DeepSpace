package frc.team3647subsystems;

import frc.robot.Robot;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import frc.team3647utility.CustomSolenoid;

public class BallIntake 
{
	private static CustomSolenoid extensionCylinder = new CustomSolenoid(Constants.ballIntakeSolinoidPin);
	private static CustomSolenoid extensionCylinder2 = new CustomSolenoid(Constants.ballIntakeSolinoidPin2);
	private static VictorSPX intakeMotor = new VictorSPX(Constants.ballMotorPin);
	
	public static void init()
	{
		intakeMotor.setInverted(false);
	}

	public static void run()
	{
		if(Robot.cargoDetection)
		{
			intake(.2);
		}
		else
		{
			intake(.6);
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

	public static void extend()
	{
		extensionCylinder.set(true);
		extensionCylinder2.set(true);
	}
	
	public static void retract()
	{
		extensionCylinder.set(false);
		extensionCylinder2.set(false);
	}

	public static void intake(double power)
	{
		setOpenLoop(power);
	}

	public static void spitOut()
	{
		setOpenLoop(-1);
	}
}