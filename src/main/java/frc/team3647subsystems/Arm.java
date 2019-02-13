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
	/**Motor controller of the arm, at 18 */
	private WPI_TalonSRX armSRX = new WPI_TalonSRX(Constants.armSRXPin);
	/**Actual motor of the arm is following armSRX at 17 */
	private CANSparkMax armNEO = new CANSparkMax(Constants.armNEOPin, CANSparkMaxLowLevel.MotorType.kBrushless);

	private VictorSPX ballHolderSPX = new VictorSPX(Constants.ballMotorPin);
	/** Piston that holds the hatch in place on the arm-hatch intake / placer */
	private Solenoid hatchHolderPiston;

	private int armVelocity;
	// public int currentPosition;

	/** All the positions the arm can be in*/
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

	/**The current (encoder) state of the arm*/
	public ArmPosition currentState;
	/**The desired (encoder) state of the arm*/
	public ArmPosition aimedState;
	
	/**Is the arm functioning with override*/
    private boolean manualOverride;

	private double overrideValue;
	
	/**Current encoder value of the arm */
	private int encoderValue = getSelectedSensorPosition();
	/**Previous encoder value of the arm */
	private int prevEncoderValue;

	/**
	 * Initialize values for Arm such as PID profiles and Feedback Sensors
	 */
    public Arm()
    {		
		// Config Sensors for Motors
		//encoder for arm  motor
		armSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs);
		armSRX.setSensorPhase(true); // if i set to false I might not need to invert gearbox motors

		// PID for motors
		armSRX.selectProfileSlot(Constants.armIdx, 0);
		armSRX.config_kP(Constants.armIdx, Constants.armkP, Constants.kTimeoutMs);
		armSRX.config_kI(Constants.armIdx, Constants.armkI, Constants.kTimeoutMs);
		armSRX.config_kD(Constants.armIdx, Constants.armkD, Constants.kTimeoutMs);
		armSRX.config_kF(Constants.armIdx, Constants.armkF, Constants.kTimeoutMs);
		armSRX.config_IntegralZone(Constants.armIdx, Constants.armIZone, Constants.kTimeoutMs);

		//arm NEO Follower Code, acrual motor that follows the SRX controller
		armNEO.follow(CANSparkMax.ExternalFollower.kFollowerPhoenix, 18);

		//Instantiate the solenoid piston used to grab the hatches from the floor intake, and from feeding location
		hatchHolderPiston = new Solenoid(5);
	}

	/**
	 * 
	 * @param positionInput 
	 * <ul>
	 * 	<li>LimitSwitchForwards
	 * 	<li>LimitSwitchBackWards
	 * 	<li>StraightForwards
	 * 	<li>StraightBackwards
	 * 	<li>StraightBackwards
	 * 	<li>StraightBackwards
	 * 	<li>CargoLevel3Front
	 * 	<li>CargoLevel3Back
	 * 	<li>HatchHandoff
	 * 	<li>HatchIntakeMovement
	 * 	<li>RobotStowed
	 * 	<li>BallHandoff
	 */
	private void setArmPosition(ArmPosition positionInput)
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
				this.setPosition(Constants.armEncoderStraightForwards);
				break;
			case StraightBackwards:
				this.setPosition(Constants.armEncoderStraightBackwards);
				break;
			case CargoLevel3Front:
				this.setPosition(Constants.armEncoderCargoLevel3Front);
				break;
			case CargoLevel3Back:
				this.setPosition(Constants.armEncoderCargoLevel3Back);
				break;
			case HatchHandoff:
				this.setPosition(Constants.armEncoderHatchHandoff);
				break;
			case HatchIntakeMovement:
				this.setPosition(Constants.armEncoderHatchIntakeMovement);
				break;
			case RobotStowed:
				this.setPosition(Constants.armEncoderRobotStowed);
				break;
			case BallHandoff:
				this.setPosition(Constants.armEncoderBallHandoff);
				break;
			default:
				break;
		}
	}

	/**
	 * The runArm methods attempts to get the arm to aimedState from current state using motion magic and stateRecognizer methods
	 */
	public void runArm(ArmPosition inputAimedState)
	{
		switch(inputAimedState)
		{
			case LimitSwitchForwards:
				setArmPosition(ArmPosition.LimitSwitchForwards);
				break;
			case LimitSwitchBackWards:
				setArmPosition(ArmPosition.HatchHandoff);
				break;
			case StraightForwards:
				setArmPosition(ArmPosition.BallHandoff);
				break;
			case StraightBackwards:
				setArmPosition(ArmPosition.HatchIntakeMovement);
				break;
			case CargoLevel3Front:
				setArmPosition(ArmPosition.RobotStowed);
				break;
			case CargoLevel3Back:
				setArmPosition(ArmPosition.CargoLevel3Back);
				break;
			case HatchHandoff:
				setArmPosition(ArmPosition.HatchHandoff);
				break;
			case HatchIntakeMovement:
				setArmPosition(ArmPosition.HatchIntakeMovement);
				break;
			case RobotStowed:
				setArmPosition(ArmPosition.RobotStowed);
				break;
			case BallHandoff:
				setArmPosition(ArmPosition.BallHandoff);
				break;
			default:
				break;
		}
	}


	/**Use Motion Magic to hold elevator at encoder position
	 * @param position : encoder value for position
	 */
    private void setPosition(double position)
    {
		//Motion Magic
        armSRX.set(ControlMode.MotionMagic, position);
	}
	
	/**Reset arm encoders using the front limit switch */
	private void resetArmForwards()
	{
		//checks if limit switch is reached
		if(!this.getForwardLimitSwitch())
		{
			// rotates arm forwards
			moveArm(-.25);
		}
		else
		{
			// Makes encoder go to 0
			resetEncoder();
			// Holds the position
			setPosition(0);
		}
	}

	/**Set arm encoders using the rear limit switch */
	private void resetArmBackwards()
	{
		// Checks if limit switch is reached
		if(!this.getReverseLimitSwitch())
		{
			//Rotates arm backwards
			moveArm(.25);
		}
		else
		{
			//Sets the encoder position as the necessary constant
			setEncoderPosition(3072); // Needs to be in constants!!
			//Holds same position using motion magic
			setPosition(3072); //As well!!
		}

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
					manualEncoderValue = encoderValue + manualAdjustment;
					encoderState = 1;
					break;
				case 1:
					setPosition(manualEncoderValue);
					break;
			}
		}
    }
    
	/* **********************FIX THIS METHOD*********************** */
	public boolean reachedDefaultPos()
	{
		return false;
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
        System.out.println("Right Elevator Current:" + armSRX.getOutputCurrent());
	}

	/**
	 * @param position in ArmPosition
	 * @return a boolean is param-position the current state of the arm
	 */
	

	public boolean isManualOverride()
	{
		return manualOverride;
	}

	/**
	 * sets previous encoder to current encoder then updates current encoder
	 */
	private void updateEncoder()
	{
		this.prevEncoderValue = this.encoderValue;
		this.encoderValue = this.getSelectedSensorPosition();
	}

	/**
	 * getter for raw encoder straight from SRX motor controller
	 */
	public int getSelectedSensorPosition()
	{
		return this.armSRX.getSelectedSensorPosition(Constants.armPIDIdx);
	}

	/**
	 * @param pos the encoder value you want the current position to be 
	 * <li>(0 is usually to reset the encoder once reached limit sensor)
	 */
	public void setEncoderPosition(int pos)
	{
		armSRX.setSelectedSensorPosition(pos, Constants.armPIDIdx, Constants.kTimeoutMs);
	}

	/**
	 * @param power [-1,1] how fast is ball arm intake roller going to spin
	 */
	public void setBallHolderPower(double power)
	{
		ballHolderSPX.set(ControlMode.PercentOutput, power);
	}


	/**
	 * @param power [.15,1] how fast are the rollers going to spin out
	 * will be used to shoot the ball to cargo ship and rocket ship
	 *  */
	public void runBallHolderOut(double power)
	{
		this.setBallHolderPower(-power);
	}

	/**
	 * used to stop ball intake rollers when ball detected in the sensors
	 */
	private void stopBallHolderMotor()
	{
		this.setBallHolderPower(0);
	}

	/**
	 * opens the hatch holder in order to grab hatches from ground intake and player station
	 */
	public void deployHatchHolderPiston()
	{
		hatchHolderPiston.set(true);
	}

	/**
	 * @return boolean for is the piston deployed
	 */
	public boolean isHatchHolderPistonDeployed()
	{
		return hatchHolderPiston.get();
	}

	/**
	 * closes the hatch holder in order to let the hatch stay on the velcro and score a point
	 */
	public void retractedHatchHolderPiston()
	{
		hatchHolderPiston.set(false);
	}

	/**Reset encoders checks if arm is at 0 position before resetting*/
	public void resetEncoder()
	{
		// Updates the encoder
		this.updateEncoder();
		if(getReverseLimitSwitch())
		{
			// Reset encoder using SRX methods
            armSRX.getSensorCollection().setQuadraturePosition(0, 10);
		}
		this.armVelocity = getVelocity();
	}

	/**updates the encoder value for the arm then returns it */
	public int getEncoder()
	{
		this.updateEncoder();
		return this.encoderValue;
	}

	/**Updates the encoder value for the arm then return the previous one */
	public int getPrevEndcoder()
	{
		this.updateEncoder();
		return this.prevEncoderValue;
	}



	/**Stops armSRX, stops arm */
	public void stopMotor()
	{
		this.armSRX.stopMotor();
	}

	/**@return selected sensor (in raw sensor units) per 100ms. See Phoenix-Documentation for how to interpret. */
	public int getVelocity()
	{
		return this.armSRX.getSelectedSensorVelocity(0);
	}

	/**@param aimedState : the state we wish the arm to be in */
	public void setAimedState(ArmPosition aimedState)
	{
		this.aimedState = aimedState;
	}

	/**@return aimedState gets the state we wish the arm to be in */
	public ArmPosition getAimedState()
	{
		return this.aimedState;
	}

	/**@return gets the current state of the arm */
	public ArmPosition getCurrentState()
	{
		return this.currentState;
	}


	private boolean getReverseLimitSwitch()
	{
		if(armSRX.getSensorCollection().isRevLimitSwitchClosed())
		{
			return false;
		}
		else
		{
			return true;
		}
    }
    
    private boolean getForwardLimitSwitch()
    {
        if(armSRX.getSensorCollection().isFwdLimitSwitchClosed())
        {
			return false;
		}
		else
		{
			return true;
		}
	}
}
