package frc.team3647subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.team3647subsystems.Elevator.ElevatorLevel;

public class Arm
{
	private WPI_TalonSRX armSRXLeader = new WPI_TalonSRX(Constants.armSRXLeaderPin);
	private CANSparkMax armNEOFollower = new CANSparkMax(Constants.armNEOFollowerPin, CANSparkMaxLowLevel.MotorType.kBrushless);
	private DigitalInput forwardLimitSwitch; // PIN NEEDED
	private DigitalInput backwardLimitSwitch; // PIN NEEDED
	private DigitalInput ballSensor; //PIN NEEDED
	private VictorSPX ballHolderSPX; //PIN NEEDED
	private Solenoid hatchHolderPiston = new Solenoid(6);

	private int armVelocity;
	// public int currentPosition;

	public enum ArmPosition
	{
		LimitSwitchForwards,
		LimitSwitchBackWards,
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
		switch(positionInput)
		{
			case LimitSwitchForwards:
				this.resetArmForwards();
				break;
			case LimitSwitchBackWards:
				this.resetArmBackwards();
				break;
			case StraightForwards:
				this.setMMMPosition(Constants.armEncoderStraightForwards);
				break;
			case StraightBackwards:
				this.setMMMPosition(Constants.armEncoderStraightBackwards);
				break;
			case CargoLevel3Front:
				this.setMMMPosition(Constants.armEncoderCargoLevel3Front);
				break;
			case CargoLevel3Back:
				this.setMMMPosition(Constants.armEncoderCargoLevel3Back);
				break;
			case HatchHandoff:
				this.setMMMPosition(Constants.armEncoderHatchHandoff);
				break;
			case HatchIntakeMovement:
				this.setMMMPosition(Constants.armEncoderHatchIntakeMovement);
				break;
			case RobotStowed:
				this.setMMMPosition(Constants.armEncoderRobotStowed);
				break;
			case BallHandoff:
				this.setMMMPosition(Constants.armEncoderBallHandoff);
				break;
			default:
				break;
		}
	}

	public void runArm()
	{
		switch(aimedState)
		{
			case LimitSwitchForwards:
				setArmPosition(ArmPosition.LimitSwitchForwards);
				if (stateRecognizer(aimedState))
					currentState = aimedState;
				break;
			case LimitSwitchBackWards:
				setArmPosition(ArmPosition.HatchHandoff);
				if (stateRecognizer(aimedState))
					currentState = aimedState;
				break;
			case StraightForwards:
				setArmPosition(ArmPosition.BallHandoff);
				if (stateRecognizer(aimedState))
					currentState = aimedState;
				break;
			case StraightBackwards:
				setArmPosition(ArmPosition.HatchIntakeMovement);
				if (stateRecognizer(aimedState))
					currentState = aimedState;
				break;
			case CargoLevel3Front:
				setArmPosition(ArmPosition.RobotStowed);
				if (stateRecognizer(aimedState))
					currentState = aimedState;
				break;
			case CargoLevel3Back:
				setArmPosition(ArmPosition.CargoLevel3Back);
				if (stateRecognizer(aimedState))
					currentState = aimedState;
				break;
			case HatchHandoff:
				setArmPosition(ArmPosition.HatchHandoff);
				if (stateRecognizer(aimedState))
					currentState = aimedState;
				break;
			case HatchIntakeMovement:
				setArmPosition(ArmPosition.HatchIntakeMovement);
				if (stateRecognizer(aimedState))
					currentState = aimedState;
				break;
			case RobotStowed:
				setArmPosition(ArmPosition.RobotStowed);
				if (stateRecognizer(aimedState))
					currentState = aimedState;
				break;
			case BallHandoff:
				setArmPosition(ArmPosition.BallHandoff);
				if (stateRecognizer(aimedState))
					currentState = aimedState;
				break;

			default:
				break;
		}
	}


    private void setMMMPosition(double position)
    {
		//Motion Magic
        armSRXLeader.set(ControlMode.MotionMagic, position);
	}
	
	private void resetArmForwards()
	{
		if(!this.getForwardLimitSwitch())
		{
			moveArm(-.25);
		}
		else
		{
			resetEncoder();
			setMMMPosition(0);
		}
	}

	private void resetArmBackwards()
	{
		if(!this.getBackwardLimitSwitch())
		{
			moveArm(.25);
		}
		else
		{
			setEncoderPosition(3072); // Needs to be in constants!!
			setMMMPosition(3072); //As well!!
		}

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
			case LimitSwitchForwards:
				return this.getForwardLimitSwitch();
			case LimitSwitchBackWards:
				return this.getBackwardLimitSwitch();
			case StraightForwards:
				if(this.stateThreshold(Constants.armEncoderStraightForwards, this.getEncoder(), Constants.armEncoderThreshold))
					return true;
				else
					return false;
			case StraightBackwards:
				if(this.stateThreshold(Constants.armEncoderStraightBackwards, this.getEncoder(), Constants.armEncoderThreshold))
					return true;
				else
					return false;
			case CargoLevel3Front:
				if(this.stateThreshold(Constants.armEncoderCargoLevel3Front, this.getEncoder(), Constants.armEncoderThreshold))
					return true;
				else
					return false;
			case CargoLevel3Back:
				if(this.stateThreshold(Constants.armEncoderCargoLevel3Back, this.getEncoder(), Constants.armEncoderThreshold))
					return true;
				else
					return false;
			case HatchHandoff:
				if(this.stateThreshold(Constants.armEncoderHatchHandoff, this.getEncoder(), Constants.armEncoderThreshold))
					return true;
				else
					return false;
			case HatchIntakeMovement:
				if(this.stateThreshold(Constants.armEncoderHatchIntakeMovement, this.getEncoder(), Constants.armEncoderThreshold))
					return true;
				else
					return false;
			case RobotStowed:
				if(this.stateThreshold(Constants.armEncoderRobotStowed, this.getEncoder(), Constants.armEncoderThreshold))
					return true;
				else
					return false;
			case BallHandoff:
				if(this.stateThreshold(Constants.armEncoderBallHandoff, this.getEncoder(), Constants.armEncoderThreshold))
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

	public void setEncoderPosition(int pos)
	{
		armSRXLeader.setSelectedSensorPosition(pos, Constants.armPIDIdx, Constants.kTimeoutMs);
	}

	public void setBallHolderPower(double power)
	{
		ballHolderSPX.set(ControlMode.PercentOutput, power);
	}

	public void runBallHolderIn()
	{
		if(this.getBallSensor())
		{
			this.stopBallHolderMotor();
		}
		else
		{
			this.setBallHolderPower(.75);
		}
	}

	public void runBallHolderOut()
	{
		this.setBallHolderPower(-.75);
	}

	private void stopBallHolderMotor()
	{
		this.setBallHolderPower(0);
	}

	public void deployHatchHolderPiston()
	{
		hatchHolderPiston.set(true);
	}

	public void retractedHatchHolderPiston()
	{
		hatchHolderPiston.set(false);
	}

	public void resetEncoder()
	{
		this.updateEncoder();
		if(reachedDefaultPos())
		{
            armSRXLeader.getSensorCollection().setQuadraturePosition(0, 10);
		}
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

	public boolean getForwardLimitSwitch()
	{
		return forwardLimitSwitch.get();
	}

	public boolean getBackwardLimitSwitch()
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

	public void setAimedState(ArmPosition aimedState)
	{
		this.aimedState = aimedState;
	}

	public ArmPosition getAimedState()
	{
		return this.aimedState;
	}

	public ArmPosition getCurrentState()
	{
		return this.currentState;
	}

	public boolean getBallSensor()
	{
		return this.ballSensor.get();
	}
}