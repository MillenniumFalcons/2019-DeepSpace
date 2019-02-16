package frc.team3647subsystems;

import frc.robot.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.*;

public class HatchGrabber
{
	public static Solenoid hatchCylinder = new Solenoid(Constants.hatchGrabberSolinoidPin);
    
    public static void runHatchGrabber(boolean joyvalue) 
	{
		if(joyvalue)
		{
			if(!hatchCylinder.get())
			{
				grabHatch();
				Timer.delay(.1);
			}
			else
			{
				releaseHatch();
				Timer.delay(.1);
			}
		}
    }

	public static void grabHatch()
	{
		hatchCylinder.set(true);
	}
	
	public static void releaseHatch()
	{
		hatchCylinder.set(false);
	}
}