package frc.team3647subsystems;

import frc.robot.*;
import frc.team3647inputs.Joysticks;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Timer;


public class HatchGrabber
{
	private static VictorSPX hatchSucker = new VictorSPX(Constants.shoppingCartSPXPin);
	private static Timer hatchIntakeTimer = new Timer();
	private static boolean hatchIn = false, flashed = false, startedTimer = false;
	
	public static void run(Joysticks coController)
	{
		// hatchIn = hatchIn();
		if(coController.rightBumper)
			grabHatch();
		else if(coController.leftBumper)
			releaseHatch();
		else if (!Robot.cargoDetection)
			runConstant();
		else
			stopMotor();
		

		// if(hatchIn)
		// {
		// 	if(!startedTimer)
		// 	{
		// 		hatchIntakeTimer.reset();
		// 		hatchIntakeTimer.start();
		// 		startedTimer = true;
		// 	}
		// 	else if(!flashed && startedTimer && hatchIntakeTimer.get() > .1)
		// 	{
		// 		Robot.mainController.setRumble(.5);
		// 		if(hatchIntakeTimer.get() > 1.1)
		// 		{
		// 			flashed = true;
		// 		}
		// 	}
		// 	if(flashed)
		// 	{
		// 		Robot.mainController.setRumble(0);
		// 	}
		// }
		// else
		// {
		// 	hatchIntakeTimer.reset();
		// 	Robot.mainController.setRumble(0);
		// 	hatchIntakeTimer.stop();
		// 	flashed = false;
		// 	startedTimer = false;
		// }
	}

	public static boolean hatchIn()
	{
		double current = Robot.pDistributionPanel.getCurrent(Constants.hatchGrabberPDPpin);
		if(current > 3.5 && current < 5.5)
		{
			return true;
		}
		return false;
	}
	
	public static void grabHatch()
	{
		hatchSucker.set(ControlMode.PercentOutput, .5);
	}
	
	public static void releaseHatch()
	{
		hatchSucker.set(ControlMode.PercentOutput, -1);
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