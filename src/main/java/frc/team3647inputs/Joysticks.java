package frc.team3647inputs;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

public class Joysticks 
{
	/**
	 * XboxController Object for Main-Driver Controller; contains all Xbox Controller Functions
	 */
	public XboxController mainController = new XboxController(0);
	/**
	 * XboxController Object for Co-Driver Controller; contains all Xbox Controller Functions
	 */
	public XboxController coController = new XboxController(1);
	
	/**
	 * Main controller Variable
	 */
	public double leftTrigger1, rightTrigger1, leftJoyStickY1, leftJoyStickX1, rightJoyStickY1, rightJoyStickX1;
	public boolean rightBumper1, leftBumper1, buttonA1, buttonB1, buttonY1, buttonX1;
	
	/**
	 * Co-Driver Controller Variable
	 */
	public double leftTrigger2, rightTrigger2, leftJoyStickY2, leftJoyStickX2, rightJoyStickY2, rightJoyStickX2;
	public boolean rightBumper2, leftBumper2, buttonA2, buttonB2, buttonY2, buttonX2, coDPadUp, coDPadDown, coDPadSide;
	/**
	 * Co-Driver Controller Variable
	 */
	public int dPadValue; //dPad degree value

	/**
	 * Used in periodic loop to update both controller values periodically
	 */
	public void updateJoysticks()
	{
		setMainContollerValues();
		setCoDriverContollerValues();
	}
	
	/**
	 * Set main controller values.
	 */
	public void setMainContollerValues()
	{
		leftBumper1		= 	mainController.getRawButton(5);
		rightBumper1 	=	mainController.getRawButton(6);
		leftTrigger1 	= 	joystickThreshold(mainController.getRawAxis(2));
		rightTrigger1 	= 	joystickThreshold(mainController.getRawAxis(3));
		buttonA1 		=	mainController.getRawButton(1);
		buttonB1 		= 	mainController.getRawButton(2);
		buttonX1 		= 	mainController.getRawButton(3);
		buttonY1 		= 	mainController.getRawButton(4);
		leftJoyStickX1 	= 	joystickThreshold(mainController.getRawAxis(0));
		leftJoyStickY1 	= 	joystickThreshold(-mainController.getRawAxis(1));
		rightJoyStickX1 = 	joystickThreshold(mainController.getRawAxis(4));
		rightJoyStickY1 = 	-joystickThreshold(mainController.getRawAxis(5));
		
	}

	/**
	 * Set co driver controller values.
	 */
	public void setCoDriverContollerValues()
	{
		leftBumper2 	=	coController.getRawButton(5);
		rightBumper2	=	coController.getRawButton(6);
		leftTrigger2 	= 	joystickThreshold(coController.getRawAxis(2));
		rightTrigger2 	= 	joystickThreshold(coController.getRawAxis(3));
		buttonA2 		=	coController.getRawButton(1);
		buttonB2 		= 	coController.getRawButton(2);
		buttonX2 		= 	coController.getRawButton(3);
		buttonY2 		= 	coController.getRawButton(4);
		leftJoyStickX2 	= 	joystickThreshold(coController.getRawAxis(0));
		leftJoyStickY2 	= 	joystickThreshold(-coController.getRawAxis(1));
		rightJoyStickX2	= 	joystickThreshold(coController.getRawAxis(4));
		rightJoyStickY2	= 	-joystickThreshold(coController.getRawAxis(5));
		dPadValue 		= 	coController.getPOV();
		setCoDPadValues();
	}

	/**
	 * 
	 * @param controller 1 = mainController, 2 = coController, 3 = both
	 */
	public void vibrate(int controller)
	{
		if(controller == 1)
		{
			mainController.setRumble(GenericHID.RumbleType.kLeftRumble,.5);
			Timer.delay(0.25);
		}
		else if(controller == 2)
		{
			coController.setRumble(GenericHID.RumbleType.kLeftRumble,.5);
			Timer.delay(0.25);
		}
		else if(controller == 3)
		{
			mainController.setRumble(GenericHID.RumbleType.kLeftRumble,.5);
			coController.setRumble(GenericHID.RumbleType.kLeftRumble,.5);
			Timer.delay(0.25);
		}
		else
		{
			System.out.println("INVALID CONTROLLER VIBRATE ID");
		}

	}
	
	/**
	 * Set co driver dPad values. 0 degrees = top, 180 = down, 90 || 270 = side
	 */
	public void setCoDPadValues()
	{
		if(dPadValue == 0)
		{
			coDPadUp = true;
			coDPadDown = false;
			coDPadSide = false;
		}
		else if(dPadValue == 180)
		{
			coDPadUp = false;
			coDPadDown = true;
			coDPadSide = false;
		}
		else if(dPadValue == 90 || dPadValue == 270)
		{
			coDPadUp = false;
			coDPadDown = false;
			coDPadSide = true;
		}
		else
		{
			coDPadUp = false;
			coDPadDown = false;
			coDPadSide = false;
		}
	}
	

	/**
	 * 
	 * @param jValue is the joystick value input
	 * @return returns joystick value if outside of joystick threshold, else returns zero
	 */
	public static double joystickThreshold(double jValue)
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
