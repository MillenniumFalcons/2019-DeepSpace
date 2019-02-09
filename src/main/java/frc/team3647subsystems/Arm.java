package frc.team3647subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.team3647subsystems.Elevator.ElevatorLevel;

public class Arm
{
	public WPI_TalonSRX armSRX = new WPI_TalonSRX(Constants.armMasterPin);
	public DigitalInput forwardLimitSwitch; // PIN NEEDED
	public DigitalInput backwardLimitSwitch; // PIN NEEDED

	public int armEncoderValue, armVelocity;
	// public int currentPosition;

	public enum ArmPosition
	{
		STRAIGHT0,
		STRAIGHT180,
		HIGHGOALFRONT,
		HIGHGOALBACK
	}
	public ArmPosition currentState;
	public ArmPosition aimedState;
	

    public DigitalInput bannerSensor = new DigitalInput(Constants.armBannerSensor);
    public boolean manualOverride;

    public double overrideValue;

	/**
	 * Initialize values for Arm such as PID profiles and Feedback Sensors
	 */
    public void armInitialization()
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
	public void setArmPosition(ArmPosition positionInput)
	{
		//Check is the arm is already at the projected position
		if(positionInput == currentState)
		{
			System.out.println("Already at " + positionInput + " position");
		}

		//check if positionInput is equal to Straight 0 degrees
		else if(positionInput == ArmPosition.STRAIGHT0 && currentState != ArmPosition.STRAIGHT0)
		{
			elevatorCheck(Constants.armStraight0);
			currentState = ArmPosition.STRAIGHT0;
		}

		//check if positionInput is equal to Straight 180 degrees
		else if(positionInput == ArmPosition.STRAIGHT180 && currentState != ArmPosition.STRAIGHT180)
		{
			elevatorCheck(Constants.armStraight180);
			currentState = ArmPosition.STRAIGHT180;
		}

		//check if positionInput is equal to High Goal Front
		else if(positionInput == ArmPosition.HIGHGOALFRONT && currentState != ArmPosition.HIGHGOALFRONT)
		{
			elevatorCheck(Constants.armHighGoalFront);
			currentState = ArmPosition.HIGHGOALFRONT;
		}

		//check if positionInput is equal to High Goal Back
		else if(positionInput == ArmPosition.HIGHGOALBACK && currentState != ArmPosition.HIGHGOALBACK)
		{
			elevatorCheck(Constants.armHighGoalBack);
			currentState = ArmPosition.HIGHGOALBACK;
		}

		else
			System.out.println("INVALID ARM POSITION");
	}

	/**
	 * Checks if the arm can move at the current elevator position
	 * @param encPositionInput arm position input
	 */
	private void elevatorCheck(int encPositionInput)
	{
		//Check if elevator is higher than the low position
		if (Robot.elevator.currentState == ElevatorLevel.MIDDLE || Robot.elevator.currentState == ElevatorLevel.MAX) 
		{
			//if true set arm to position without moving elevator
			setArmEncPosition(encPositionInput);
		}
		else
		{
			//else set elevator to middle position
			Robot.elevator.setElevatorLevel(ElevatorLevel.MIDDLE);
			//then set arm position to input position
			setArmEncPosition(encPositionInput);
		}

	}



    private void setArmEncPosition(double position)
    {
		//Motion Magic
        armSRX.set(ControlMode.MotionMagic, position);
    }

    public void stopArm()
    {
        //Stop Elevator
        armSRX.stopMotor();
    }

    public void moveArm(double speed)
    {
		//Percent Output [-1,1]
        armSRX.set(ControlMode.PercentOutput, speed);
    }    

    public int encoderState;
	public int manualEncoderValue;
	public int manualAdjustment;

    public void moveManual(double jValue)
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
    
	public void setArmEncoder()
	{
        if(reachedDefaultPos())
		{
            resetArmEncoders();
		}
		armEncoderValue = armSRX.getSelectedSensorPosition(0);
		armVelocity = armSRX.getSelectedSensorVelocity(0);
	}
	
	public void resetArmEncoders()
	{
        armSRX.getSensorCollection().setQuadraturePosition(0, 10);
	}

	public boolean reachedDefaultPos()
	{
        if(bannerSensor.get())
            return false;
        else
            return true;

	}

	public void setManualOverride(double jValue)
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

	public void printArmEncoders()
    {
        System.out.println("Elevator Encoder Value: " + armEncoderValue + "Elevator Velocity: " + armVelocity);
	}

	public void bannerSensorTriggered()
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

    public void printArmCurrent()
    {
        System.out.println("Right Elevator Current:" + armSRX.getOutputCurrent());
	}

	public static boolean stateRecognizer(ArmPosition position)
	{
		int currentArmEncoder = Robot.encoders.getArmEncoder();
		switch(position)
		{
			case STRAIGHT0:
				if(stateThreshold(Constants.armStraight0, currentArmEncoder, Constants.armEncoderThreshold))
					return true;
				else
					return false;
			case STRAIGHT180:
				if(stateThreshold(Constants.armStraight180, currentArmEncoder, Constants.armEncoderThreshold))
					return true;
				else
					return false;
			case HIGHGOALFRONT:
				if(stateThreshold(Constants.armHighGoalFront, currentArmEncoder, Constants.armEncoderThreshold))
					return true;
				else
					return false;
			case HIGHGOALBACK:
				if(stateThreshold(Constants.armHighGoalBack, currentArmEncoder, Constants.armEncoderThreshold))
					return true;
				else
					return false;
			default:
				return false;
		}
	}

	private static boolean stateThreshold(double constant, double encoder, double threshold)
	{
		if((constant + threshold) > encoder && (constant - threshold) < encoder)
		{
			return true;
		}
		else
		{
			return false;
		}
	}

}