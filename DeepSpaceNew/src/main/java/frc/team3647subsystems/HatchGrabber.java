package frc.team3647subsystems;

import frc.robot.*;
import frc.team3647inputs.Joysticks;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;


public class HatchGrabber
{
	private static VictorSPX hatchSucker = new VictorSPX(Constants.shoppingCartSPXPin);
	
	public static void runHatchGrabber(Joysticks coController)
	{
		if(coController.rightBumper)
			grabHatch();
		else if(coController.leftBumper)
			releaseHatch();
		else if (!BallShooter.cargoDetection())
			runConstant();
		else
			stopMotor();		
	}
	
	private static void grabHatch()
	{
		hatchSucker.set(ControlMode.PercentOutput, .5);
	}
	
	private static void releaseHatch()
	{
		hatchSucker.set(ControlMode.PercentOutput, -1);
	}

	private static void runConstant()
	{
		hatchSucker.set(ControlMode.PercentOutput, .23);
	}
	private static void stopMotor()
	{
		hatchSucker.set(ControlMode.PercentOutput, 0);
	}
}