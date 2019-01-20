package frc.team3647subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class Arm
{
    public static WPI_TalonSRX armSRX = new WPI_TalonSRX(Constants.armMaster);

    /***************** Need Fixing Beyond This Comment ***************/

	public static int armEncoderValue, armVelocity;
	

    public static DigitalInput bannerSensor = new DigitalInput(Constants.elevatorBannerSensor);
    public static boolean bottomGoal, middleGoal, highGoal, lowerScale, moving, manualOverride, originalPositionButton;

    public static double overrideValue;


    public static void armInitialization()
    {

	}

    public static void setArmPosition(double position)
    {
		//Motion Magic
        armSRX.set(ControlMode.MotionMagic, position);
    }

    public static void stopArm()
    {
        //Stop Elevator
        armSRX.stopMotor();
    }

    public static void moveArm(double speed)
    {
		//Percent Output [-1,1]
        armSRX.set(ControlMode.PercentOutput, speed);
    }    

    public static int encoderState;
	public static int manualEncoderValue;
	public static int manualAdjustment;

    public static void moveManual(double jValue)
	{
		if(jValue > 0)
		{
			moveArm(overrideValue * 0.65);
			manualAdjustment = 1500;
			encoderState = 0;
		}
		else if(jValue < 0)
		{
			moveArm(overrideValue * 0.2);
			manualAdjustment = 0;
			encoderState = 0;
		}
		else
		{
			switch(encoderState)
			{
				case 0:
					manualEncoderValue = armEncoderValue + manualAdjustment;
					encoderState = 1;
					break;
				case 1:
					setArmPosition(manualEncoderValue);
					break;
			}
		}
    }
    
	public static void setArmEncoder()
	{
        if(reachedBottom())
		{
            resetArmEncoders();
		}
		armEncoderValue = armSRX.getSelectedSensorPosition(0);
		armVelocity = armSRX.getSelectedSensorVelocity(0);
	}
	
	public static void resetArmEncoders()
	{
        Elevator.GearboxMaster.getSensorCollection().setQuadraturePosition(0, 10);
	}

	public static boolean reachedBottom()
	{
        if(bannerSensor.get())
            return false;
        else
            return true;

	}

	public static void setArmButtons(boolean Button)
	{
		//supposedly 8 levels
	}

	public static void setManualOverride(double jValue)
	{
        if(Math.abs(jValue) <.2 )
		{
			manualOverride = false;
		}
		else
		{
            overrideValue = jValue;
			manualOverride = true;
		}
	}

	public static void testArmEncoders()
    {
        System.out.println("Elevator Encoder Value: " + armEncoderValue + "Elevator Velocity: " + armVelocity);
	}

	public static void testBannerSensor()
    {
        if(reachedBottom())
        {
            System.out.println("Banner Sensor Triggered!");
        }
        else
        {
            System.out.println("Banner Sensor Not Triggered!");
        }
    }

    public static void testArmCurrent()
    {
        System.out.println("Right Elevator Current:" + armSRX.getOutputCurrent());
	}


}