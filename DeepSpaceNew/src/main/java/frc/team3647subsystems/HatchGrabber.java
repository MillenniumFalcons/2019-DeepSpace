package frc.team3647subsystems;

import frc.robot.*;
import frc.team3647inputs.Joysticks;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.*;

public class HatchGrabber
{
	public static Solenoid hatchCylinder = new Solenoid(Constants.hatchGrabberSolinoidPin);
	public static VictorSPX hatchSucker = new VictorSPX(Constants.shoppingCartSPXPin);
	// public static PowerDistributionPanel pdp = new PowerDistributionPanel();
    
    // public static void runHatchGrabber(Joystick joystick) 
	// {
	// 	if()
	// 		grabHatch();
	// 	else
	// 		grabHatch();
	// }
	
	public static void runHatchGrabber(Joysticks coController)
	{
		if(coController.rightBumper)
			grabHatch();
		else if(coController.leftBumper)
			releaseHatch();
		else
			runConstant();

		
	}
	public static void grabHatch()
	{
		// hatchCylinder.set(false);
		hatchSucker.set(ControlMode.PercentOutput, .5);
	}
	
	public static void releaseHatch()
	{
		hatchSucker.set(ControlMode.PercentOutput, -1);
	}

	public static void runConstant()
	{
		hatchSucker.set(ControlMode.PercentOutput, .23);
	}
	public static void stopMotor()
	{
		hatchSucker.set(ControlMode.PercentOutput, 0);
	}
}