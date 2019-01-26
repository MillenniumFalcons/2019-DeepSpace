package frc.team3647subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class Arm
{
    public static WPI_TalonSRX armSRX = new WPI_TalonSRX(Constants.armMaster);

	public static int armEncoderValue, armVelocity;
	public static int currentPosition;
	

    public static DigitalInput bannerSensor = new DigitalInput(Constants.armBannerSensor);
    public static boolean manualOverride;

    public static double overrideValue;

	/**
	 * Initialize values for Arm such as PID profiles and Feedback Sensors
	 */
    public static void armInitialization()
    {

		// Config Sensors for Motors
		armSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs);
		armSRX.setSensorPhase(true); // if i set to false I might not need to invert gearbox motors

		// PID for motors
		armSRX.selectProfileSlot(Constants.armProfile1, 0);
		armSRX.config_kP(Constants.armProfile1, Constants.armkP, Constants.kTimeoutMs);
		armSRX.config_kI(Constants.armProfile1, Constants.armkI, Constants.kTimeoutMs);
		armSRX.config_kD(Constants.armProfile1, Constants.armkD, Constants.kTimeoutMs);
		armSRX.config_kF(Constants.armProfile1, Constants.armkF, Constants.kTimeoutMs);
		armSRX.config_IntegralZone(Constants.armProfile1, Constants.armIZone, Constants.kTimeoutMs);

	}

	/**
	 * 
	 * @param positionInput 1 = Straight 0; 2 = Straight 180; 3 = High Goal Front; 4 = High Goal Back; Position other # = error
	 */
	public static void setArmPosition(int positionInput)
	{

		//Check is the arm is already at the projected position
		if(positionInput == currentPosition)
		{
			System.out.println("Already at " + positionInput + " position");
		}

		//check if positionInput is equal to Straight 0 degrees
		else if(positionInput == 1 && currentPosition != 1)
		{
			elevatorCheck(Constants.armStraight0);
			currentPosition = 1;
		}

		//check if positionInput is equal to Straight 180 degrees
		else if(positionInput == 2 && currentPosition != 2)
		{
			elevatorCheck(Constants.armStraight180);
			currentPosition = 2;
		}

		//check if positionInput is equal to High Goal Front
		else if(positionInput == 3 && currentPosition != 3)
		{
			elevatorCheck(Constants.armHighGoalFront);
			currentPosition = 3;
		}

		//check if positionInput is equal to High Goal Back
		else if(positionInput == 4 && currentPosition != 4)
		{
			elevatorCheck(Constants.armHighGoalBack);
			currentPosition = 4;
		}

		else
			System.out.println("INVALID ARM POSITION");
	}

	/**
	 * Checks if the arm can move at the current elevator position
	 * @param encPositionInput arm position input
	 */
	private static void elevatorCheck(int encPositionInput)
	{
		//Check if elevator is higher than the low position
		if (Elevator.currentLevel == 2 || Elevator.currentLevel == 3) 
		{
			//if true set arm to position without moving elevator
			setArmEncPosition(encPositionInput);
		}
		else
		{
			//else set elevator to middle position
			Elevator.setElevatorLevel(2);
			//then set arm position to input position
			setArmEncPosition(encPositionInput);
		}

	}



    private static void setArmEncPosition(double position)
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
					setArmEncPosition(manualEncoderValue);
					break;
			}
		}
    }
    
	public static void setArmEncoder()
	{
        if(reachedDefaultPos())
		{
            resetArmEncoders();
		}
		armEncoderValue = armSRX.getSelectedSensorPosition(0);
		armVelocity = armSRX.getSelectedSensorVelocity(0);
	}
	
	public static void resetArmEncoders()
	{
        armSRX.getSensorCollection().setQuadraturePosition(0, 10);
	}

	public static boolean reachedDefaultPos()
	{
        if(bannerSensor.get())
            return false;
        else
            return true;

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

	public static void printArmEncoders()
    {
        System.out.println("Elevator Encoder Value: " + armEncoderValue + "Elevator Velocity: " + armVelocity);
	}

	public static void bannerSensorTriggered()
    {
        if(reachedDefaultPos())
        {
            System.out.println("Banner Sensor Triggered!");
        }
        else
        {
            System.out.println("Banner Sensor Not Triggered!");
        }
    }

    public static void printArmCurrent()
    {
        System.out.println("Right Elevator Current:" + armSRX.getOutputCurrent());
	}


}