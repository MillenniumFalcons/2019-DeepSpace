package frc.team3647inputs;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;

public class Joysticks 
{
	public Joystick mainController = new Joystick(0);
	public Joystick coController = new Joystick(1);
	public GenericHID dPad = new Joystick(1);
	
	//Main controller Variables
	public double leftTrigger1, rightTrigger1, leftJoyStickY1, leftJoyStickX1, rightJoyStickY1, rightJoyStickX1;
	public boolean rightBumper1, leftBumper1, buttonA1, buttonB1, buttonY1, buttonX1;
	
	//Co-Driver Controller Variables
	public double leftTrigger2, rightTrigger2, leftJoyStickY2, leftJoyStickX2, rightJoyStickY2, rightJoyStickX2;
	public boolean rightBumper2, leftBumper2, buttonA2, buttonB2, buttonY2, buttonX2, dPadUp, dPadDown, dPadSide;
	public int dPadValue;
	
	public void setMainContollerValues()
	{
		leftBumper1		= 	mainController.getRawButton(5);
		rightBumper1 	=	mainController.getRawButton(6);
		leftTrigger1 	= 	fixJoystickValue(mainController.getRawAxis(2));
		rightTrigger1 	= 	fixJoystickValue(mainController.getRawAxis(3));
		buttonA1 		=	mainController.getRawButton(1);
		buttonB1 		= 	mainController.getRawButton(2);
		buttonX1 		= 	mainController.getRawButton(3);
		buttonY1 		= 	mainController.getRawButton(4);
		leftJoyStickX1 	= 	fixJoystickValue(mainController.getRawAxis(0));
		leftJoyStickY1 	= 	fixJoystickValue(-mainController.getRawAxis(1));
		rightJoyStickX1 = 	fixJoystickValue(mainController.getRawAxis(4));
		rightJoyStickY1 = 	-fixJoystickValue(mainController.getRawAxis(5));
		
	}
	
	public void setCoDriverContollerValues()
	{
		leftBumper2 	=	coController.getRawButton(5);
		rightBumper2	=	coController.getRawButton(6);
		leftTrigger2 	= 	fixJoystickValue(coController.getRawAxis(2));
		rightTrigger2 	= 	fixJoystickValue(coController.getRawAxis(3));
		buttonA2 		=	coController.getRawButton(1);
		buttonB2 		= 	coController.getRawButton(2);
		buttonX2 		= 	coController.getRawButton(3);
		buttonY2 		= 	coController.getRawButton(4);
		leftJoyStickX2 	= 	fixJoystickValue(coController.getRawAxis(0));
		leftJoyStickY2 	= 	fixJoystickValue(-coController.getRawAxis(1));
		rightJoyStickX2	= 	fixJoystickValue(coController.getRawAxis(4));
		rightJoyStickY2	= 	-fixJoystickValue(coController.getRawAxis(5));
		dPadValue 		= 	dPad.getPOV();
		setDPadValues();
	}
	
	public void setDPadValues()
	{
		if(dPadValue == 0)
		{
			dPadUp = true;
			dPadDown = false;
			dPadSide = false;
		}
		else if(dPadValue == 180)
		{
			dPadUp = false;
			dPadDown = true;
			dPadSide = false;
		}
		else if(dPadValue == 90 || dPadValue == 270)
		{
			dPadUp = false;
			dPadDown = false;
			dPadSide = true;
		}
		else
		{
			dPadUp = false;
			dPadDown = false;
			dPadSide = false;
		}
	}
	
	public static double fixJoystickValue(double jValue)
	{
		if(jValue < .13 && jValue > -.13)
		{
			return 0;
		}
		else
		{
			return 1 * jValue;
		}
	}
}
