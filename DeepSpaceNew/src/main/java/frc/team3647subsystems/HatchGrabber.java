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
    
    // public static void runHatchGrabber(Joystick joystick) 
	// {
	// 	if()
	// 		grabHatch();
	// 	else
	// 		grabHatch();
	// }
	
	public static void runHatchGrabber(Joysticks mainController, Joysticks coController)
	{
		if(mainController.leftTrigger > .3 || coController.rightBumper)
			grabHatch();
		else if(mainController.rightTrigger > .3)
			releaseHatch();
		else
			stopMotor();

		
	}
	public static void grabHatch()
	{
		// hatchCylinder.set(false);
		hatchSucker.set(ControlMode.PercentOutput, 1);
	}
	
	public static void releaseHatch()
	{
		hatchSucker.set(ControlMode.PercentOutput, -1);
	}

	public static void stopMotor()
	{
		hatchSucker.set(ControlMode.PercentOutput, 0);
	}
}