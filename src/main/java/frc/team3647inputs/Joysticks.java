package frc.team3647inputs;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;

public class Joysticks 
{
	public Joystick mainController = new Joystick(0);
	public Joystick coController = new Joystick(1);
	public GenericHID dPad = new Joystick(1);
	
	// Main controller Variables
	public double leftTrigger, rightTrigger, leftJoySticky, leftJoyStickx, rightJoySticky, rightJoyStickx;
	public boolean rightBumper, leftBumper, buttonA, buttonB, buttonY, buttonX;
	
	//Co-Driver Controller Variables
	public double leftTrigger1, rightTrigger1, leftJoySticky1, leftJoyStickx1, rightJoySticky1, rightJoyStickx1;
	public boolean rightBumper1, leftBumper1, buttonA1, buttonB1, buttonY1, buttonX1, dPadUp, dPadDown, dPadSide;
	public int dPadValue;
	
	public void setMainContollerValues()
	{
		//rightBumper =	mainController.getRawButton(6);
		leftBumper =	mainController.getRawButton(5);
		leftTrigger = fixJoystickValue(mainController.getRawAxis(2));
		buttonA =	mainController.getRawButton(1);
		buttonB = mainController.getRawButton(2);
		rightTrigger = fixJoystickValue(mainController.getRawAxis(3));
		buttonY = mainController.getRawButton(4);
		leftJoySticky = fixJoystickValue(-mainController.getRawAxis(1));
		leftJoyStickx = fixJoystickValue(mainController.getRawAxis(0));
		rightJoyStickx = fixJoystickValue(mainController.getRawAxis(4));
		rightJoySticky = -fixJoystickValue(mainController.getRawAxis(5));
		buttonX = mainController.getRawButton(3);
	}
	
	public void setCoDriverContollerValues()
	{
		rightBumper1 =	coController.getRawButton(6);
		leftBumper1 =	coController.getRawButton(5);
		leftTrigger1 = fixJoystickValue(coController.getRawAxis(2));
		buttonA1 =	coController.getRawButton(1);
		buttonB1 = coController.getRawButton(2);
		rightTrigger1 = fixJoystickValue(coController.getRawAxis(3));
		buttonY1 = coController.getRawButton(4);
		leftJoySticky1 = fixJoystickValue(-coController.getRawAxis(1));
		leftJoyStickx1 = fixJoystickValue(coController.getRawAxis(0));
		rightJoyStickx1 = fixJoystickValue(coController.getRawAxis(4));
		rightJoySticky1 = -fixJoystickValue(coController.getRawAxis(5));
		buttonX1 = coController.getRawButton(3);
		dPadValue = dPad.getPOV();
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
