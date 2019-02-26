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
	public static CANDigitalInput revNeoLimitSwitch; 
	public static CANDigitalInput fwdNeoLimitSwitch;

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


		// Reverse and forward limit switch from the NEO
		revNeoLimitSwitch = armNEO.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
		fwdNeoLimitSwitch = armNEO.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);

		// PID for motors
		armSRX.selectProfileSlot(Constants.armPID, 0);
		armSRX.config_kP(Constants.armPID, Constants.armPIDF[0], Constants.kTimeoutMs);
		armSRX.config_kI(Constants.armPID, Constants.armPIDF[1], Constants.kTimeoutMs);
		armSRX.config_kD(Constants.armPID, Constants.armPIDF[2], Constants.kTimeoutMs);
		armSRX.config_kF(Constants.armPID, Constants.armPIDF[3], Constants.kTimeoutMs);
		armSRX.configMotionCruiseVelocity(Constants.kArmSRXCruiseVelocity, Constants.kTimeoutMs);	
		armSRX.configMotionAcceleration(Constants.kArmSRXAcceleration, Constants.kTimeoutMs);

		// The NEO takes the Motor-output in percent from the SRX and since SRX values are using motion-magic, it "follows" the SRX
		followerThread = new Notifier(()->
		{
			//armNEO.set(armSRX.getMotorOutputPercent()); //set straight %output
			armNEO.set(armSRX.getMotorOutputVoltage()/12); //set to voltage that srx is output on a scale of -1 to 1
		});
		// Thread runs at 10ms, so setting and getting output from SRX, has no latency with PID values
		followerThread.startPeriodic(0.005);
		armEncoderValue = 11000;
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
		FWDLIMITSWITCH,
		REVLIMITSWITCH,
		FLATFORWARDS,
		FLATBACKWARDS,
		CARGOL3FRONT,
		CARGOL3BACK,
		HATCHHANDOFF,
		STOWED,
		VERTICALSTOWED,
		CARGOHANDOFF,
		MANUAL, 
		STOPPED,
		CLIMB
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


	/**
	 * The main arm method, switches the aimedState variable to determine which position to move to next
	 */
	public static void runArm()
	{
		// if(Math.abs(armEncoderVelocity) > 200 && BallShooter.cargoDetection())
		// {
		// 	BallShooter.intakeCargo(.1);
		// }
		setArmEncoder();
		updateLivePosition();
		// setManualOverride(controller.rightJoyStickY);
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
				case FLATFORWARDS:
					moveToFlatForwards();
					break;
				case FLATBACKWARDS:
					moveToFlatBackwards();
					break;
				case CARGOL3FRONT:
					moveToCargoL3Front();
					break;
				case CARGOL3BACK:
					moveToCargoL3Back();
					break;
				case HATCHHANDOFF:
					moveToHatchHandoff();
					break;
				case STOWED:
					moveToStowed();
					break;
				case VERTICALSTOWED:
					moveToVerticalStowed();
					break;
				case CARGOHANDOFF:
					moveToCargoHandoff();
					break;
				case CLIMB:
					moveToClimbPos();
					break;
				default:
					System.out.println("Unreachable Arm Position");
					break;
			}
		}
	}

	public static void updateLivePosition()
	{
		if(getRevLimitSwitch())
		{
			currentState = ArmPosition.REVLIMITSWITCH;
			lastState = ArmPosition.REVLIMITSWITCH;
		}
		else if(getFwdLimitSwitch())
		{
			currentState = ArmPosition.FWDLIMITSWITCH;
			lastState = ArmPosition.REVLIMITSWITCH; // DIFFERENT?
		}
		else if(positionThreshold(Constants.armSRXCargoHandoff))
		{
			currentState = ArmPosition.CARGOHANDOFF;
			lastState = ArmPosition.CARGOHANDOFF;
		}
		else if(positionThreshold(Constants.armSRXHatchHandoff))
		{
			currentState = ArmPosition.HATCHHANDOFF;
			lastState = ArmPosition.HATCHHANDOFF;
		}
		else if(positionThreshold(Constants.armSRXFlatForwards))
		{
			currentState = ArmPosition.FLATFORWARDS;
			lastState = ArmPosition.FLATFORWARDS;
		}
		else if(positionThreshold(Constants.armSRXFlatBackwards))
		{
			currentState = ArmPosition.FLATBACKWARDS;
			lastState = ArmPosition.FLATBACKWARDS;
		}
		else if(positionThreshold(Constants.armSRXCargoL3Front))
		{
			currentState = ArmPosition.CARGOL3FRONT;
			lastState = ArmPosition.CARGOL3FRONT;
		}
		else if(positionThreshold(Constants.armSRXCargoL3Back))
		{
			currentState = ArmPosition.CARGOL3BACK;
			lastState = ArmPosition.CARGOL3BACK;
		}
		else if(positionThreshold(Constants.armSRXStowed))
		{
			currentState = ArmPosition.STOWED;
			lastState = ArmPosition.STOWED;
		}
		else if(positionThreshold(Constants.armSRXVerticalStowed))
		{
			currentState = ArmPosition.VERTICALSTOWED;
			lastState = ArmPosition.VERTICALSTOWED;
		}
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
		{
			manualOverride = false;
		}
	}
	// Limit switch based movement: -------
	public static void moveToFwdLimitSwitch()
	{
		if(!getFwdLimitSwitch())
		{
			// rotates arm forwards
			setOpenLoop(.2);
		}
		else
		{
			//setEncoderValue(Constants.armSRXFwdLimitSwitch);
			//setPosition(Constants.armSRXFwdLimitSwitch);
			stopArm();
		}
	}

	public static void moveToRevLimitSwitch()
	{
		if(!getRevLimitSwitch())
		{
			setOpenLoop(-.3);
		}
		else
		{
			resetArmEncoder();
			setPosition(0);
		}
	}
	//---------------------------------------

	//MotionMagic movement-------------------
	public static void setPosition(int position)
	{
		// System.out.println("Going to encoder position: " + position);
		// Motion Magic
		armSRX.set(ControlMode.MotionMagic, position);
	}

	public static void moveToFlatForwards()
	{
		setPosition(Constants.armSRXFlatForwards);
	}

	public static void moveToFlatBackwards()
	{
		setPosition(Constants.armSRXFlatBackwards);
	}

	public static void moveToCargoL3Front()
	{
		setPosition(Constants.armSRXCargoL3Front);
	}

	public static void moveToCargoL3Back()
	{
		setPosition(Constants.armSRXCargoL3Back);
	}

	public static void moveToHatchHandoff()
	{
		setPosition(Constants.armSRXHatchHandoff);
	}
	
	public static void moveToStowed()
	{
		setPosition(Constants.armSRXStowed);
	}

	public static void moveToVerticalStowed()
	{
		setPosition(Constants.armSRXVerticalStowed);
	}

	public static void moveToCargoHandoff()
	{
		setPosition(Constants.armSRXCargoHandoff);
	}

	private static void moveToClimbPos()
	{
		if(SeriesStateMachine.inThreshold(Arm.armEncoderValue, Constants.armSRXClimb, 500))
			stopArm();
		else
			setPosition(Constants.armSRXClimb);
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

	public static void setOpenLoop(double power)
	{
		//Percent Output
		armSRX.set(ControlMode.PercentOutput, power);
		//armNEO.set(power);
	}

	public static void stopArm()
	{
		armSRX.stopMotor();
		armNEO.stopMotor();
	}
	//-------------------------------------------------



	public static boolean positionThreshold(double constant)
	{
		if((constant + Constants.kArmSRXPositionThreshold) > armEncoderValue && (constant - Constants.kArmSRXPositionThreshold) < armEncoderValue)
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	public static ArmPosition livePosition()
	{
		return ArmPosition.FLATBACKWARDS;
	}



	// Encoder Methods ----------------------------------------
	public static void setArmEncoder()
	{
		// When the arm rotates all the way to the back encoder resets
		if(getRevLimitSwitch())
		{
			resetArmEncoder();
		}
		else if(getFwdLimitSwitch())
		{
			//setEncoderValue(Constants.armSRXFwdLimitSwitch);
		}

		// Gets encoder from SRX
		armEncoderValue = armSRX.getSelectedSensorPosition(0);
		armEncoderVelocity = armSRX.getSelectedSensorVelocity(0);
		// Gets difference between aimed encoder value and current encoder value
		armEncoderCCL = armSRX.getClosedLoopError(0);
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
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	//----------------------------------------------------------


	// Limit switch methods ------------------------------------
	public static boolean getRevLimitSwitch()
	{
		//Limit switch connected to the NEO
		// return Robot.coController.leftJoyStickPress;
		// return armSRX.getSensorCollection().isRevLimitSwitchClosed();
		return revNeoLimitSwitch.get();
	}

	public static boolean getFwdLimitSwitch()
	{
		//Limit switch connected to the NEO
		return fwdNeoLimitSwitch.get();
		// return armSRX.getSensorCollection().isRevLimitSwitchClosed();
	}
	//----------------------------------------------------------
	
	public static void setToBrake()
	{
		armNEO.setIdleMode(IdleMode.kBrake);
	}
	public static void printArmEncoders()
    {
        System.out.println("\nArm Encoder Value: " + armEncoderValue + "\nArm Velocity: " + armEncoderVelocity);
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

	public static void closeFollowerThread()
	{
		followerThread.stop();
	}

}