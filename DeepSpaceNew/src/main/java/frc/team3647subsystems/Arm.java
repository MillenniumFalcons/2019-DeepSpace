package frc.team3647subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Constants;
import frc.team3647inputs.*;

public class Arm
{
	public static ArmPosition aimedState;

	public static WPI_TalonSRX armSRX = new WPI_TalonSRX(Constants.armSRXPin);	
	public static CANSparkMax armNEO = new CANSparkMax(Constants.armNEOPin, CANSparkMaxLowLevel.MotorType.kBrushless);
	public static CANDigitalInput revNeoLimitSwitch = armNEO.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen); 
	public static CANDigitalInput fwdNeoLimitSwitch = armNEO.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen); ;

	public static int encoderError, encoderValue, encoderVelocity;

	public static double overrideValue;
	public static boolean manualOverride;
		
    public static void init()
	{
		aimedState = ArmPosition.STOPPED;
		
		initSensors();
	}
	
	public static void initSensors()
	{
		armSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs);
		armSRX.setSensorPhase(true);
		armSRX.setInverted(false);

		armNEO.restoreFactoryDefaults();
		armNEO.setIdleMode(IdleMode.kBrake);
		armNEO.enableVoltageCompensation(12);//max voltage of 12v to scale output better

		configPIDFMM(Constants.armPIDF[0], Constants.armPIDF[1], Constants.armPIDF[2], Constants.armPIDF[3], 
					Constants.kArmSRXCruiseVelocity, Constants.kArmSRXAcceleration);

	}

	public static void configPIDFMM(double p, double i, double d, double f, int vel, int accel)
	{
		armSRX.selectProfileSlot(Constants.armPID, 0);
		armSRX.config_kP(Constants.armPID, p, Constants.kTimeoutMs);		
		armSRX.config_kI(Constants.armPID, i, Constants.kTimeoutMs);	
		armSRX.config_kD(Constants.armPID, d, Constants.kTimeoutMs);
		armSRX.config_kF(Constants.armPID, f, Constants.kTimeoutMs);
		//Motion Magic Constants
		armSRX.configMotionCruiseVelocity(vel, Constants.kTimeoutMs);
        armSRX.configMotionAcceleration(accel, Constants.kTimeoutMs);
	}
	
	public enum ArmPosition
	{
		FWDLIMITSWITCH(Constants.armSRXFwdLimitSwitch),
		REVLIMITSWITCH(-1),
		FLATFORWARDS(Constants.armSRXFlatForwards),
		FLATVISIONFORWARDS(Constants.armSRXForwardVision),
		FLATVISIONBACKWARDS(Constants.armSRXBackwardVision),
		FLATBACKWARDS(Constants.armSRXFlatBackwards),
		CARGOL3FRONT(Constants.armSRXCargoL3Front),
		CARGOL3BACK(Constants.armSRXCargoL3Back),
		HATCHHANDOFF(Constants.armSRXHatchHandoff),
		STOWED(Constants.armSRXStowed),
		VERTICALSTOWED(Constants.armSRXVerticalStowed),
		CARGOHANDOFF(Constants.armSRXCargoHandoff),
		MANUAL(-1), 
		STOPPED(-1),
		CLIMB(Constants.armSRXClimb),
		VISIONF(Constants.armSRXForwardVision),
		VISIONB(Constants.armSRXBackwardVision), 
		CARGOSHIPBACKWARDS(Constants.armSRXCargoShipBack), 
		CARGOSHIPFORWARDS(Constants.armSRXCargoShipFront);

		public int encoderVal;
		ArmPosition(int encoderVal)
		{
			this.encoderVal = encoderVal;
		}
	}

	public static void armNEOFollow()
	{
		armNEO.set(armSRX.getMotorOutputVoltage()/12); //set to voltage that srx is output on a scale of -1 to 1
	}

	/**
	 * The main arm method, switches the aimedState variable to determine which position to move to next
	 */
	public static void run()
	{
		updateEncoder();
		// updateLivePosition();
		if(aimedState != null) //check if aimed state has a value
		{
			switch(aimedState)
			{
				case MANUAL:
					// if(!manualOverride)
					// 	overrideValue = 0;
					// moveManual(overrideValue);
					break;
				case STOPPED:
					Arm.stop();
					break;
				case FWDLIMITSWITCH:
					moveToFwdLimitSwitch();
					break;
				case REVLIMITSWITCH:
					moveToRevLimitSwitch();
					break;
				default:
					if(aimedState.encoderVal != -1)
						setPosition(aimedState.encoderVal);
					break;
			}
		}
	}


	
	// Limit switch based movement: -------
	public static void moveToFwdLimitSwitch()
	{
		if(!getFwdLimitSwitch())
			setOpenLoop(.2);
		else
			stop();
	}

	public static void moveToRevLimitSwitch()
	{
		if(!getRevLimitSwitch())
			setOpenLoop(-.5);
		else
		{
			resetEncoder();
			setPosition(0);
		}
	}
	//---------------------------------------
	//MotionMagic movement-------------------
	private static void setPosition(int position)
	{
		// Motion Magic
		try{ armSRX.set(ControlMode.MotionMagic, position); }
		catch(NullPointerException e)
		{
			armSRX = new WPI_TalonSRX(Constants.armSRXPin);
			armSRX.set(ControlMode.MotionMagic, position);
		}
	}
	//-----------------------------------------
	
	public static void setOpenLoop(double demand)
	{
		try{ armSRX.set(ControlMode.PercentOutput, demand); }
		catch(NullPointerException e){ armSRX = new WPI_TalonSRX(Constants.armSRXPin); }
	}

	public static void stop()
	{
		armSRX.stopMotor();
		armNEO.stopMotor();
	}
	//-------------------------------------------------

	private static boolean positionThreshold(double constant)
	{
		return  (constant + Constants.kArmSRXPositionThreshold) > encoderValue && 
				(constant - Constants.kArmSRXPositionThreshold) < encoderValue;
	}

	public static boolean reachedState(ArmPosition nAimedState)
	{
		if(aimedState != null)
			return positionThreshold(nAimedState.encoderVal);
		return false;
	}

	public static boolean reachedAimedState()
	{
		return reachedState(aimedState);
	}
	
	// Encoder Methods ----------------------------------------
	public static void updateEncoder()
	{
		// When the arm rotates all the way to the back encoder resets
		// Gets encoder from SRX
		try
		{
			encoderValue = armSRX.getSelectedSensorPosition(0);
			encoderVelocity = armSRX.getSelectedSensorVelocity(0);
			encoderError = armSRX.getClosedLoopError(0); // Gets difference between aimed encoder value and current encoder value
		}
		catch(NullPointerException e)
		{
			armSRX = new WPI_TalonSRX(Constants.armSRXPin);
			armSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs);
		}
	}

	private static void setEncoderValue(int encoderValue)
	{
		armSRX.setSelectedSensorPosition(encoderValue, 0, Constants.kTimeoutMs);
	}

	public static void resetEncoder()
	{
		armSRX.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
	}

	public static boolean isEncoderInThreshold()
	{
		// Checks the difference between the aimed encoder value and the current encoder value
		// If difference less than a constant, assume current and aimed positions are the same
		if(Math.abs(encoderError) < 80)
			return true;
		return false;
	}
	//----------------------------------------------------------
	// Limit switch methods ------------------------------------
	public static boolean getRevLimitSwitch()
	{
		try{return revNeoLimitSwitch.get();}
		catch(NullPointerException e)
		{
			armNEO = new CANSparkMax(Constants.armNEOPin, CANSparkMaxLowLevel.MotorType.kBrushless);
			revNeoLimitSwitch = armNEO.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
			return revNeoLimitSwitch.get();
		}
	}

	public static boolean getFwdLimitSwitch()
	{
		try{return fwdNeoLimitSwitch.get();}
		catch(NullPointerException e)
		{
			armNEO = new CANSparkMax(Constants.armNEOPin, CANSparkMaxLowLevel.MotorType.kBrushless);
			fwdNeoLimitSwitch = armNEO.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
			return fwdNeoLimitSwitch.get();
		}
	}
	//----------------------------------------------------------
	
	// Manual Movement-------------------------
	public static void setManualController(Joysticks controller)
	{
		setManualOverride(controller.leftJoyStickY);
		if(manualOverride)
			aimedState = ArmPosition.MANUAL;
		else if(controller.buttonA)
			aimedState = ArmPosition.VERTICALSTOWED;
		else if(controller.buttonY)
			aimedState = ArmPosition.STOWED;
		 else if(controller.buttonB)
			aimedState = ArmPosition.HATCHHANDOFF;
		else if(controller.buttonX)
			aimedState = ArmPosition.CARGOHANDOFF;
		 else if(controller.leftBumper)
			aimedState = ArmPosition.FLATBACKWARDS;
		 else if(controller.rightBumper)
			aimedState = ArmPosition.FLATFORWARDS;	
		 else if(controller.dPadRight)
			aimedState = ArmPosition.FWDLIMITSWITCH;
		else if(controller.dPadLeft)
			aimedState = ArmPosition.REVLIMITSWITCH;
		 else if(controller.dPadUp)
			aimedState = ArmPosition.CARGOL3BACK;
		 else if(controller.dPadDown)
			aimedState = ArmPosition.CARGOL3FRONT;
	}

	public static void setArmManualControl(Joysticks controller)
	{
		setManualOverride(controller.rightJoyStickX);
	}
	public static void setManualOverride(double jValue)
	{
		if(Math.abs(jValue) > .1) //deadzone
		{
			manualOverride = true;
			overrideValue = jValue;
			aimedState = ArmPosition.MANUAL;
		} 
		else 
			manualOverride = false;
	}
	private static int encoderState, manualAdjustment, manualEncoderValue;

    public static void moveManual(double jValue)
	{
		updateEncoder();
		// updateLivePosition();
		if(jValue > 0)
		{
			setOpenLoop(overrideValue * 0.5);
			manualAdjustment = 0;
			encoderState = 0;
		}
		else if(jValue < 0)
		{
			setOpenLoop(overrideValue * 0.5);
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

	public static void setToBrake()
	{
		armNEO.setIdleMode(IdleMode.kBrake);
	}
	public static void printEncoders()
    {
        System.out.println("\nArm Encoder Value: " + encoderValue);
	}

	public static void printLimitSwitches()
	{
		System.out.println("\nRev Limit Switch: " + getRevLimitSwitch() + "\nFwd Limit Swtich: " + getFwdLimitSwitch());
	}

    public static void printCurrent()
    {
        System.out.println("ArmCurrent: " + armNEO.getOutputCurrent());
	}

	public static void printPercentOutput()
	{
		System.out.println("Elevator percent output: " + armSRX.getMotorOutputPercent());
	}

	public static void disableArm()
	{
		stop();
		armNEO.setIdleMode(IdleMode.kCoast);
	}
}