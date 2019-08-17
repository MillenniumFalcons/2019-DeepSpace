package frc.team3647subsystems;

import frc.robot.*;
import frc.team3647inputs.Joysticks;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Timer;


public class HatchGrabber
{
	private static VictorSPX hatchSucker = new VictorSPX(Constants.shoppingCartSPXPin);
	private static double maxCurrent = 7;

	public static void init()
	{
		
		hatchSucker.configOpenloopRamp(5); // 5 seconds
		
	}
	public static void run(Joysticks coController)
	{
		if(coController.rightBumper)
			grabHatch();
		else if(coController.leftBumper)
			releaseHatch();
		else if (!Robot.cargoDetection)
			runConstant();
		else
			stopMotor();
	}


	public static boolean hatchIn()
	{
		double current = Robot.pDistributionPanel.getCurrent(Constants.hatchGrabberPDPpin);
		return current > 3.5 && current < 5.5;
	}
	
	private static double limitCurrent(double motorConst, double currentConst)
	{
		double current = Robot.pDistributionPanel.getCurrent(Constants.hatchGrabberPDPpin);
		// System.out.println(current);
		if(current == 0)
		{
			return motorConst;
		}
		else if(current < currentConst)
		{
			return motorConst;
		}
		return (currentConst / current) * motorConst;
		
	}

	public static void grabHatch()
	{
		// hatchSucker.set(ControlMode.PercentOutput, .6);
		hatchSucker.set(ControlMode.PercentOutput, limitCurrent(.6, 3.5));
	}
	
	public static void releaseHatch()
	{
		hatchSucker.set(ControlMode.PercentOutput, -limitCurrent(1, 15));
	}

	public static void runConstant()
	{
		hatchSucker.set(ControlMode.PercentOutput, .2);
	}
	public static void stopMotor()
	{
		hatchSucker.set(ControlMode.PercentOutput, 0);
	}
}