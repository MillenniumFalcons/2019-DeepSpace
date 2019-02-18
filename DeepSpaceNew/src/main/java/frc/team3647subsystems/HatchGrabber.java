package frc.team3647subsystems;

import frc.robot.*;
import edu.wpi.first.wpilibj.*;

public class HatchGrabber
{
	public static Solenoid hatchCylinder = new Solenoid(Constants.hatchGrabberSolinoidPin);
    
    public static void runHatchGrabber(boolean joyvalue) 
	{
		if(joyvalue)
			releaseHatch();
		else
			grabHatch();
    }

	public static void grabHatch()
	{
		hatchCylinder.set(false);
	}
	
	public static void releaseHatch()
	{
		hatchCylinder.set(true);
	}
}