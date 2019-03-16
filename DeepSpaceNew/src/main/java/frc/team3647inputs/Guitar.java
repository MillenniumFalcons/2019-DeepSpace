package frc.team3647inputs;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;

public class Guitar 
{
	/**
	 * Joystick Object for Guitar Controller
	 */
	private Joystick guitarController;
		
	/**
	 * Controller Variables
	 */
	public boolean fret1Up, fret1Down, fret2Up, fret2Down, fret3Up, fret3Down, strumUp, strumDown, stow;


	public Guitar(int controllerPin)
	{
		guitarController = new Joystick(controllerPin);
	}

	
	/**
	 * Set Controller values.
	 */
	public void setGuitarControllerValues()
	{
		fret1Up 	=	guitarController.getRawButton(0);
		fret2Up 	= 	guitarController.getRawButton(1);
		fret3Up 	= 	guitarController.getRawButton(2);
		fret1Down 	=	guitarController.getRawButton(3);
		fret2Down	= 	guitarController.getRawButton(4);
        fret3Down 	= 	guitarController.getRawButton(5);
		stow        =   guitarController.getRawButton(7);
        strumUp     =   guitarController.getRawButton(12);
        strumDown   =   guitarController.getRawButton(13);
    }
    
}