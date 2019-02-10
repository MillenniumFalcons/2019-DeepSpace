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
	private WPI_TalonSRX armSRXLeader = new WPI_TalonSRX(Constants.armSRXLeaderPin);
	/**Actual motor of the arm is following armSRX at 17 */
	private CANSparkMax armNEOFollower = new CANSparkMax(Constants.armNEOFollowerPin, CANSparkMaxLowLevel.MotorType.kBrushless);
	
	/**The limit switch in the front of the robot (where the ground ball intake is), is at 135deg from pos x-axis
	 * <p> PIN NEEDED
	*/
	private DigitalInput forwardLimitSwitch =new DigitalInput(Constants.armLimitSwitchForward); 

	/**The limit switch in the back of the robot (where the hatch ground intake is), is at 45deg from pos x-axis
	* <p> PIN NEEDED
	*/
	private DigitalInput backwardLimitSwitch = new DigitalInput(Constants.armLimitSwitchBackward);

	/**
	 * On the arm-ball-intake, detectes if the ball is present or not
	 * <p>PIN NEEDED
	 */
	private DigitalInput ballSensor = new DigitalInput(Constants.armBallBannerSensor);


	/**Motor that moves the arm-ball-intake rollers that suck the ball from ground intake
	* <p>PIN NEEDED
	*/
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
		armSRXLeader.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs);
		armSRXLeader.setSensorPhase(true); // if i set to false I might not need to invert gearbox motors

		// PID for motors
		armSRXLeader.selectProfileSlot(Constants.armProfile1, 0);
		armSRXLeader.config_kP(Constants.armProfile1, Constants.armkP, Constants.kTimeoutMs);
		armSRXLeader.config_kI(Constants.armProfile1, Constants.armkI, Constants.kTimeoutMs);
		armSRXLeader.config_kD(Constants.armProfile1, Constants.armkD, Constants.kTimeoutMs);
		armSRXLeader.config_kF(Constants.armProfile1, Constants.armkF, Constants.kTimeoutMs);
		armSRXLeader.config_IntegralZone(Constants.armProfile1, Constants.armIZone, Constants.kTimeoutMs);

		//arm NEO Follower Code, acrual motor that follows the SRX controller
		armNEOFollower.follow(CANSparkMax.ExternalFollower.kFollowerPhoenix, 18);

		//Instantiate the solenoid piston used to grab the hatches from the floor intake, and from feeding location
		hatchHolderPiston = new Solenoid(6);
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
				this.setMMPosition(Constants.armEncoderStraightForwards);
				break;
			case StraightBackwards:
				this.setMMPosition(Constants.armEncoderStraightBackwards);
				break;
			case CargoLevel3Front:
				this.setMMPosition(Constants.armEncoderCargoLevel3Front);
				break;
			case CargoLevel3Back:
				this.setMMPosition(Constants.armEncoderCargoLevel3Back);
				break;
			case HatchHandoff:
				this.setMMPosition(Constants.armEncoderHatchHandoff);
				break;
			case HatchIntakeMovement:
				this.setMMPosition(Constants.armEncoderHatchIntakeMovement);
				break;
			case RobotStowed:
				this.setMMPosition(Constants.armEncoderRobotStowed);
				break;
			case BallHandoff:
				this.setMMPosition(Constants.armEncoderBallHandoff);
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
				if (stateRecognizer(inputAimedState))
					currentState = inputAimedState;
				break;
			case LimitSwitchBackWards:
				setArmPosition(ArmPosition.HatchHandoff);
				if (stateRecognizer(inputAimedState))
					currentState = inputAimedState;
				break;
			case StraightForwards:
				setArmPosition(ArmPosition.BallHandoff);
				if (stateRecognizer(inputAimedState))
					currentState = inputAimedState;
				break;
			case StraightBackwards:
				setArmPosition(ArmPosition.HatchIntakeMovement);
				if (stateRecognizer(inputAimedState))
					currentState = inputAimedState;
				break;
			case CargoLevel3Front:
				setArmPosition(ArmPosition.RobotStowed);
				if (stateRecognizer(inputAimedState))
					currentState = inputAimedState;
				break;
			case CargoLevel3Back:
				setArmPosition(ArmPosition.CargoLevel3Back);
				if (stateRecognizer(inputAimedState))
					currentState = inputAimedState;
				break;
			case HatchHandoff:
				setArmPosition(ArmPosition.HatchHandoff);
				if (stateRecognizer(inputAimedState))
					currentState = inputAimedState;
				break;
			case HatchIntakeMovement:
				setArmPosition(ArmPosition.HatchIntakeMovement);
				if (stateRecognizer(inputAimedState))
					currentState = inputAimedState;
				break;
			case RobotStowed:
				setArmPosition(ArmPosition.RobotStowed);
				if (stateRecognizer(inputAimedState))
					currentState = inputAimedState;
				break;
			case BallHandoff:
				setArmPosition(ArmPosition.BallHandoff);
				if (stateRecognizer(inputAimedState))
					currentState = inputAimedState;
				break;
			default:
				break;
		}
	}


	/**Use Motion Magic to hold elevator at encoder position
	 * @param position : encoder value for position
	 */
    private void setMMPosition(double position)
    {
		//Motion Magic
        armSRXLeader.set(ControlMode.MotionMagic, position);
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
			setMMPosition(0);
		}
	}

	/**Set arm encoders using the rear limit switch */
	private void resetArmBackwards()
	{
		// Checks if limit switch is reached
		if(!this.getBackwardLimitSwitch())
		{
			//Rotates arm backwards
			moveArm(.25);
		}
		else
		{
			//Sets the encoder position as the necessary constant
			setEncoderPosition(3072); // Needs to be in constants!!
			//Holds same position using motion magic
			setMMPosition(3072); //As well!!
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
					setMMPosition(manualEncoderValue);
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
        System.out.println("Right Elevator Current:" + armSRXLeader.getOutputCurrent());
	}

	/**
	 * @param position in ArmPosition
	 * @return a boolean is param-position the current state of the arm
	 */
	public boolean stateRecognizer(ArmPosition position)
	{
		switch(position)
		{
			// is the required position the top most position in the front (therefore we can use limit switches to determine it)
			case LimitSwitchForwards:
				return this.getForwardLimitSwitch();
			// is the required position the top most position in the back (therefore we can use limit switches to determine it)
			case LimitSwitchBackWards:
				return this.getBackwardLimitSwitch();
			//Completely horizotal, compares against constant encoder values found guessing and checking 
			case StraightForwards:
				if(this.stateThreshold(Constants.armEncoderStraightForwards, this.getEncoder(), Constants.armEncoderThreshold))
					return true;
				else
					return false;
			//Completely horizotal, compares against constant encoder values found guessing and checking 
			case StraightBackwards:
				if(this.stateThreshold(Constants.armEncoderStraightBackwards, this.getEncoder(), Constants.armEncoderThreshold))
					return true;
				else
					return false;
			// A little below front limit switch
			case CargoLevel3Front:
				if(this.stateThreshold(Constants.armEncoderCargoLevel3Front, this.getEncoder(), Constants.armEncoderThreshold))
					return true;
				else
					return false;
			// A little below rear limit switch		
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

	/**
	 * 
	 * @param constant the desired value of the arm encoder
	 * @param encoder the current value of the arm encoder
	 * @param threshold What is the leniency of the arm position
	 * @return boolean is current encoder value within threshold of supposed state
	 */
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
		return this.armSRXLeader.getSelectedSensorPosition(Constants.armPIDIdx);
	}

	/**
	 * @param pos the encoder value you want the current position to be 
	 * <li>(0 is usually to reset the encoder once reached limit sensor)
	 */
	public void setEncoderPosition(int pos)
	{
		armSRXLeader.setSelectedSensorPosition(pos, Constants.armPIDIdx, Constants.kTimeoutMs);
	}

	/**
	 * @param power [-1,1] how fast is ball arm intake roller going to spin
	 */
	public void setBallHolderPower(double power)
	{
		ballHolderSPX.set(ControlMode.PercentOutput, power);
	}
	
	/**
	 * @param power [.15,1] how fast is the roller going to spin
	 * stops motor if detects ball with ball sensor, otherwise runs motor inside
	 * Used to suck a ball from the ground intake
	 */
	public void runBallHolderIn(double power)
	{
		if(this.getBallSensor())
		{
			this.stopBallHolderMotor();
		}
		else
		{
			this.setBallHolderPower(power);
		}
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
		if(getBackwardLimitSwitch())
		{
			// Reset encoder using SRX methods
            armSRXLeader.getSensorCollection().setQuadraturePosition(0, 10);
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

	/**Is limit switch at the front is reached */
	public boolean getForwardLimitSwitch()
	{
		//True when activated
		return !this.forwardLimitSwitch.get();
	}

	/**Is limit switch at the back reached */
	public boolean getBackwardLimitSwitch()
	{
		//True when activated
		return !this.backwardLimitSwitch.get();
	}

	/**Stops armSRX, stops arm */
	public void stopMotor()
	{
		this.armSRXLeader.stopMotor();
	}

	/**@return selected sensor (in raw sensor units) per 100ms. See Phoenix-Documentation for how to interpret. */
	public int getVelocity()
	{
		return this.armSRXLeader.getSelectedSensorVelocity(0);
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

	/**@return boolean for does the robot think there is a ball or not */
	public boolean getBallSensor()
	{
		return this.ballSensor.get();
	}
}