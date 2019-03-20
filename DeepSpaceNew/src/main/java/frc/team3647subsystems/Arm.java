package frc.team3647subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Notifier;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.team3647inputs.*;

public class Arm
{
	public static ArmPosition currentState, lastState, aimedState;

	public static WPI_TalonSRX armSRX = new WPI_TalonSRX(Constants.armSRXPin);	
	public static CANSparkMax armNEO = new CANSparkMax(Constants.armNEOPin, CANSparkMaxLowLevel.MotorType.kBrushless);
	public static CANDigitalInput revNeoLimitSwitch = armNEO.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen); 
	public static CANDigitalInput fwdNeoLimitSwitch = armNEO.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen); ;

	public static int armEncoderCCL, armEncoderValue, armEncoderVelocity;

	public static double overrideValue;
	public static boolean manualOverride;
	
	static Notifier followerThread;
	
    public static void armInitialization()
	{
		currentState = null;
		aimedState = null;
		lastState = null;
		//Encoder for arm motor
		armSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs);
		armSRX.setSensorPhase(true);
		armSRX.setInverted(false);

		armNEO.restoreFactoryDefaults();
		armNEO.setIdleMode(IdleMode.kBrake);
		armNEO.enableVoltageCompensation(12);//max voltage of 12v to scale output better

		// PID for motors
		armSRX.selectProfileSlot(Constants.armPID, 0);
		armSRX.config_kP(Constants.armPID, Constants.armPIDF[0], Constants.kTimeoutMs);
		armSRX.config_kI(Constants.armPID, Constants.armPIDF[1], Constants.kTimeoutMs);
		armSRX.config_kD(Constants.armPID, Constants.armPIDF[2], Constants.kTimeoutMs);
		armSRX.config_kF(Constants.armPID, Constants.armPIDF[3], Constants.kTimeoutMs);
		armSRX.configMotionCruiseVelocity(Constants.kArmSRXCruiseVelocity, Constants.kTimeoutMs);
		armSRX.configMotionAcceleration(Constants.kArmSRXAcceleration, Constants.kTimeoutMs);
	}
	
	public static void armInitSensors()
	{
		armSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs);
		armSRX.setSensorPhase(true);
		armSRX.setInverted(false);

		armNEO.restoreFactoryDefaults();
		armNEO.setIdleMode(IdleMode.kBrake);
		armNEO.enableVoltageCompensation(12);//max voltage of 12v to scale output better

		// PID for motors
		armSRX.selectProfileSlot(Constants.armPID, 0);
		armSRX.config_kP(Constants.armPID, Constants.armPIDF[0], Constants.kTimeoutMs);
		armSRX.config_kI(Constants.armPID, Constants.armPIDF[1], Constants.kTimeoutMs);
		armSRX.config_kD(Constants.armPID, Constants.armPIDF[2], Constants.kTimeoutMs);
		armSRX.config_kF(Constants.armPID, Constants.armPIDF[3], Constants.kTimeoutMs);
		armSRX.configMotionCruiseVelocity(Constants.kArmSRXCruiseVelocity, Constants.kTimeoutMs);
		armSRX.configMotionAcceleration(Constants.kArmSRXAcceleration, Constants.kTimeoutMs);
	}

	public static void configurePIDFMM(double p, double i, double d, double f, int vel, int accel)
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
	/**
	 * The main arm method, switches the aimedState variable to determine which position to move to next
	 */
	public static void runArm()
	{
		setArmEncoder();
		updateLivePosition();
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
					Arm.stopArm();
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

	public static void updateLivePosition()
	{
		if(aimedState != null)
		{
			if(positionThreshold(aimedState.encoderVal))
				currentState = aimedState;
		}
		else if(getFwdLimitSwitch())
			currentState = ArmPosition.FWDLIMITSWITCH;
		else if(positionThreshold(Constants.armSRXCargoHandoff))
			currentState = ArmPosition.CARGOHANDOFF;
		else if(positionThreshold(Constants.armSRXHatchHandoff))
			currentState = ArmPosition.HATCHHANDOFF;
		else if(positionThreshold(Constants.armSRXFlatForwards))
			currentState = ArmPosition.FLATFORWARDS;
		else if(positionThreshold(Constants.armSRXFlatBackwards))
			currentState = ArmPosition.FLATBACKWARDS;
		else if(positionThreshold(Constants.armSRXCargoL3Front))
			currentState = ArmPosition.CARGOL3FRONT;
		else if(positionThreshold(Constants.armSRXCargoL3Back))
			currentState = ArmPosition.CARGOL3BACK;
		else if(positionThreshold(Constants.armSRXStowed))
			currentState = ArmPosition.STOWED;
		else if(positionThreshold(Constants.armSRXVerticalStowed))
			currentState = ArmPosition.VERTICALSTOWED;
		else if(positionThreshold(Constants.armSRXBackwardVision))
			currentState = ArmPosition.VISIONB;
		else if(positionThreshold(Constants.armSRXForwardVision))
			currentState = ArmPosition.VISIONF;
		else
			currentState = null;
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
	// Limit switch based movement: -------
	public static void moveToFwdLimitSwitch()
	{
		if(!getFwdLimitSwitch())
			setOpenLoop(.2);
		else
			stopArm();
	}

	public static void moveToRevLimitSwitch()
	{
		if(!getRevLimitSwitch())
			setOpenLoop(-.5);
		else
		{
			resetArmEncoder();
			setPosition(0);
		}
	}
	//---------------------------------------
	//MotionMagic movement-------------------
	private static void setPosition(int position)
	{
		// Motion Magic
		armSRX.set(ControlMode.MotionMagic, position);
	}
	//-----------------------------------------
	// Manual Movement-------------------------
	private static int encoderState, manualAdjustment, manualEncoderValue;

    public static void moveManual(double jValue)
	{
		setArmEncoder();
		updateLivePosition();
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
					manualEncoderValue = armEncoderValue + manualAdjustment;
					encoderState = 1;
					break;
				case 1:
					setPosition(manualEncoderValue);
					break;
			}
		}
	}

	public static void setOpenLoop(double demand)
	{
		armSRX.set(ControlMode.PercentOutput, demand);
	}

	public static void stopArm()
	{
		armSRX.stopMotor();
		armNEO.stopMotor();
	}
	//-------------------------------------------------

	public static boolean positionThreshold(double constant)
	{
		return  (constant + Constants.kArmSRXPositionThreshold) > armEncoderValue && 
				(constant - Constants.kArmSRXPositionThreshold) < armEncoderValue;
	}

	public static ArmPosition livePosition()
	{
		return ArmPosition.FLATBACKWARDS;
	}

	// Encoder Methods ----------------------------------------
	public static void setArmEncoder()
	{
		// When the arm rotates all the way to the back encoder resets
		// Gets encoder from SRX
		armEncoderValue = armSRX.getSelectedSensorPosition(0);
		armEncoderVelocity = armSRX.getSelectedSensorVelocity(0);
		armEncoderCCL = armSRX.getClosedLoopError(0); // Gets difference between aimed encoder value and current encoder value
	}

	public static void setEncoderValue(int encoderValue)
	{
		armSRX.setSelectedSensorPosition(encoderValue, 0, Constants.kTimeoutMs);
	}

	public static void resetArmEncoder()
	{
		armSRX.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
	}

	public static boolean isEncoderInThreshold()
	{
		// Checks the difference between the aimed encoder value and the current encoder value
		// If difference less than a constant, assume current and aimed positions are the same
		if(Math.abs(armEncoderCCL) < 80)
			return true;
		return false;
	}
	//----------------------------------------------------------
	// Limit switch methods ------------------------------------
	public static boolean getRevLimitSwitch()
	{
		return revNeoLimitSwitch.get();
	}

	public static boolean getFwdLimitSwitch()
	{
		return fwdNeoLimitSwitch.get();
	}
	//----------------------------------------------------------
	
	public static void setToBrake()
	{
		armNEO.setIdleMode(IdleMode.kBrake);
	}
	public static void printArmEncoders()
    {
        System.out.println("\nArm Encoder Value: " + armEncoderValue);// + "\nArm Velocity: " + armEncoderVelocity);
	}

	public static void printArmLimitSwitches()
	{
		System.out.println("\nRev Limit Switch: " + getRevLimitSwitch() + "\nFwd Limit Swtich: " + getFwdLimitSwitch());
	}

    public static void printArmCurrent()
    {
        System.out.println("ArmCurrent: " + armNEO.getOutputCurrent());
	}

	public static void printPercentOutput()
	{
		System.out.println("Elevator percent output: " + armSRX.getMotorOutputPercent());
	}

	public static void disableArm()
	{
		stopArm();
		armNEO.setIdleMode(IdleMode.kCoast);
	}
}