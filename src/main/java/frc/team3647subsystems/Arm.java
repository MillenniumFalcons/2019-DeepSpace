package frc.team3647subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.team3647subsystems.Elevator.ElevatorLevel;

public class Arm
{
	private WPI_TalonSRX armSRXLeader = new WPI_TalonSRX(Constants.armSRXLeaderPin);
	private CANSparkMax armNEOFollower = new CANSparkMax(Constants.armNEOFollowerPin, CANSparkMaxLowLevel.MotorType.kBrushless);
	private DigitalInput forwardLimitSwitch; // PIN NEEDED
	private DigitalInput backwardLimitSwitch; // PIN NEEDED

	private int armVelocity;
	// public int currentPosition;

	public enum ArmPosition
	{
		StraightForwards,
		StraightBackwards,
		CargoLevel3Front,
		CargoLevel3Back,
		HatchHandoff,
		HatchIntakeMovement,
		RobotStowed,
		BallHandoff,
	}
	public ArmPosition currentState;
	public ArmPosition aimedState;
	
    private boolean manualOverride;

	private double overrideValue;
	
	private int encoderValue = getSelectedSensorPosition();
	private int prevEncoderValue;

	/**
	 * Initialize values for Arm such as PID profiles and Feedback Sensors
	 */
    public Arm()
    {		
		// Config Sensors for Motors
		armSRXLeader.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs);
		armSRXLeader.setSensorPhase(true); // if i set to false I might not need to invert gearbox motors

		// PID for motors
		armSRXLeader.selectProfileSlot(Constants.armProfile1, 0);
		armSRXLeader.config_kP(Constants.armProfile1, Constants.armkP, Constants.kTimeoutMs);
		armSRXLeader.config_kI(Constants.armProfile1, Constants.armkI, Constants.kTimeoutMs);
		armSRXLeader.config_kD(Constants.armProfile1, Constants.armkD, Constants.kTimeoutMs);
		armSRXLeader.config_kF(Constants.armProfile1, Constants.armkF, Constants.kTimeoutMs);
		armSRXLeader.config_IntegralZone(Constants.armProfile1, Constants.armIZone, Constants.kTimeoutMs);

		//arm NEO Follower Code
		armNEOFollower.follow(CANSparkMax.ExternalFollower.kFollowerPhoenix, 18);
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
		else if(positionInput == ArmPosition.StraightForwards && currentState != ArmPosition.StraightBackwards)
		{
			elevatorCheck(Constants.armEncoderStraightForwards);
			currentState = ArmPosition.StraightForwards;
		}

		//check if positionInput is equal to Straight 180 degrees
		else if(positionInput == ArmPosition.StraightBackwards && currentState != ArmPosition.StraightBackwards)
		{
			elevatorCheck(Constants.armEncoderStraightBackwards);
			currentState = ArmPosition.StraightBackwards;
		}

		//check if positionInput is equal to High Goal Front
		else if(positionInput == ArmPosition.CargoLevel3Front && currentState != ArmPosition.CargoLevel3Front)
		{
			elevatorCheck(Constants.armEncoderCargoLevel3Front);
			currentState = ArmPosition.CargoLevel3Front;
		}

		//check if positionInput is equal to High Goal Back
		else if(positionInput == ArmPosition.CargoLevel3Back && currentState != ArmPosition.CargoLevel3Back)
		{
			//elevatorCheck(Constants.elevatorCur);
			currentState = ArmPosition.CargoLevel3Back;
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
			setMMMPosition(encPositionInput);
		}
		else
		{
			//else set elevator to middle position
			Robot.elevator.setElevatorPosition(ElevatorLevel.MIDDLE);
			//then set arm position to input position
			setMMMPosition(encPositionInput);
		}

	}

    private void setMMMPosition(double position)
    {
		//Motion Magic
        armSRXLeader.set(ControlMode.MotionMagic, position);
    }

    public void moveArm(double speed)
    {
		//Percent Output [-1,1]
        armSRXLeader.set(ControlMode.PercentOutput, speed);
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
					manualEncoderValue = encoderValue + manualAdjustment;
					encoderState = 1;
					break;
				case 1:
					setMMMPosition(manualEncoderValue);
					break;
			}
		}
    }
    
	/* **********************FIX THIS METHOD*********************** */
	public boolean reachedDefaultPos()
	{
		return true;
	}
	/* **********************FIX THIS METHOD*********************** */

	public void setManualOverride(double jValue)
	{
        if(Math.abs(jValue) <.2 )
		{
			this.manualOverride = false;
		}
		else
		{
            this.overrideValue = jValue;
			this.manualOverride = true;
		}
	}

	public void printArmEncoders()
    {
        System.out.println("Elevator Encoder Value: " + encoderValue + "Elevator Velocity: " + armVelocity);
	}

	public void bannerSensorTriggered()
    {
        if(this.reachedDefaultPos())
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
        System.out.println("Right Elevator Current:" + armSRXLeader.getOutputCurrent());
	}

	public boolean stateRecognizer(ArmPosition position)
	{
		switch(position)
		{
			case StraightForwards:
				if(this.stateThreshold(Constants.armEncoderStraightForwards, this.encoderValue, Constants.armEncoderThreshold))
					return true;
				else
					return false;
			case StraightBackwards:
				if(this.stateThreshold(Constants.armEncoderStraightBackwards, this.encoderValue, Constants.armEncoderThreshold))
					return true;
				else
					return false;
			case CargoLevel3Front:
				if(this.stateThreshold(Constants.armEncoderCargoLevel3Front, this.encoderValue, Constants.armEncoderThreshold))
					return true;
				else
					return false;
			case CargoLevel3Back:
				if(this.stateThreshold(Constants.armEncoderCargoLevel3Back, this.encoderValue, Constants.armEncoderThreshold))
					return true;
				else
					return false;
			default:
				return false;
		}
	}

	private boolean stateThreshold(double constant, double encoder, double threshold)
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

	public boolean isManualOverride()
	{
		return manualOverride;
	}

	private void updateEncoder()
	{
		this.prevEncoderValue = this.encoderValue;
		this.encoderValue = this.getSelectedSensorPosition();
	}

	public int getSelectedSensorPosition()
	{
		this.updateEncoder();
		return this.armSRXLeader.getSelectedSensorPosition(Constants.armPIDIdx);
	}

	public void setEncoderPosition()
	{
		armSRXLeader.setSelectedSensorPosition(0);
	}

	public void resetEncoder()
	{
		this.updateEncoder();
		if(reachedDefaultPos())
		{
            armSRXLeader.getSensorCollection().setQuadraturePosition(0, 10);
		}
		armSRXLeader.setSelectedSensorPosition(0, Constants.armPIDIdx, Constants.kTimeoutMs);
		this.armVelocity = getVelocity();
	}

	public int getEncoder()
	{
		this.updateEncoder();
		return this.encoderValue;
	}

	public int getPrevEndcoder()
	{
		this.updateEncoder();
		return this.prevEncoderValue;
	}

	public boolean getForwardSensor()
	{
		return forwardLimitSwitch.get();
	}

	public boolean getBackwardSensor()
	{
		return backwardLimitSwitch.get();
	}

	public void stopMotor()
	{
		armSRXLeader.stopMotor();
	}

	public int getVelocity()
	{
		return this.armSRXLeader.getSelectedSensorVelocity(0);
	}
}