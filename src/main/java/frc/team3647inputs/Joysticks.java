package frc.team3647inputs;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

public class Joysticks 
{
	/**
	 * XboxController Object for Main-Driver Controller; contains all Xbox Controller Functions
	 */
	private XboxController controller;
	
	// to get dPad degrees
	private GenericHID dPad;
	
	/**
	 * Main controller Variable
	 */
	public double leftTrigger, rightTrigger, leftJoyStickY, leftJoyStickX, rightJoyStickY, rightJoyStickX;
	public boolean rightBumper, leftBumper, buttonA, buttonB, buttonY, buttonX, dPadDown, dPadLeft, dPadRight, dPadUp;
	public boolean rightJoyStickPress, leftJoyStickPress;
	
	/**
	 * Co-Driver Controller Variable
	 */
	public int dPadValue; //dPad degree value


	public Joysticks(int controllerPin)
	{
		controller = new XboxController(controllerPin);
		dPad = new XboxController(controllerPin);
	}

	
	/**
	 * Set main controller values.
	 */
	public void setMainContollerValues()
	{
		leftBumper		= 	controller.getBumper(XboxController.Hand.kLeft);
		rightBumper 	=	controller.getRawButton(6);
		leftTrigger 	= 	joystickThreshold(controller.getRawAxis(2));
		rightTrigger 	= 	joystickThreshold(controller.getRawAxis(3));
		buttonA 		=	controller.getRawButton(1);
		buttonB 		= 	controller.getRawButton(2);
		buttonX 		= 	controller.getRawButton(3);
		buttonY 		= 	controller.getRawButton(4);
		leftJoyStickX 	= 	joystickThreshold(controller.getRawAxis(0));
		leftJoyStickY 	= 	joystickThreshold(-controller.getRawAxis(1));
		rightJoyStickX = 	joystickThreshold(controller.getRawAxis(4));
		rightJoyStickY = 	-joystickThreshold(controller.getRawAxis(5));
		rightJoyStickPress = controller.getRawButton(9);
		leftJoyStickPress = controller.getRawButton(9);
		dPadValue = dPad.getPOV();
		setDPadValues();
		// System.out.println("Updating controller!");
	}


	/**
	 * @param controller 1 = mainController, 2 = coController, 3 = both
	 * @param power Rumble power from 0 to 1
	 * @param times amount of times to rumble
	 */
	public void vibrate(int controller, double power, int times)
	{
		double delay = 0.1;
		for(int i = 0; i < times; i++) 
		{
			setRumble(power);
			Timer.delay(delay);
			setRumble(0);
			Timer.delay(delay);
		}
	}

	/**
	 * easier way to set rumble for both side of controller
	 * @param joystick object
	 * @param power power of rumble from 0 to 1
	 */
	private void setRumble(double power)
	{
		controller.setRumble(GenericHID.RumbleType.kLeftRumble, power);
		controller.setRumble(GenericHID.RumbleType.kRightRumble, power);
	}
	
	/**
	 * Set co driver dPad values. 0 degrees = top, 180 = down, 90 right 270 == left
	 */
	public void setDPadValues()
	{
		if(dPadValue == 0)
		{
			dPadUp = true;
			dPadDown = false;
			dPadLeft = false;
			dPadRight = false;
		}
		if(dPadValue == 180)
		{
			dPadUp = false;
			dPadDown = true;
			dPadLeft = false;
			dPadRight = false;	
		}
		if(dPadValue == 90)
		{
			dPadUp = false;
			dPadDown = false;
			dPadLeft = false;
			dPadRight = true;
		}
		if(dPadValue == 270)
		{
			dPadUp = false;
			dPadDown = false;
			dPadLeft = true;
			dPadRight = false;
		}
		if(dPadValue == -1)
		{
			dPadUp = false;
			dPadDown = false;
			dPadLeft = false;
			dPadRight = false;
		}
	}
	

	/**
	 * 
	 * @param jValue is the joystick value input
	 * @return returns joystick value if outside of joystick threshold, else returns zero
	 */
	public static double joystickThreshold(double jValue)
	{
		if(Math.abs(jValue) < .09)
		{
			return 0;
		}
		else
		{
			return 1 * jValue;
		}
	}
}
