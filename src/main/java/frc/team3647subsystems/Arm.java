package frc.team3647subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;
import frc.robot.*;
import frc.team3647inputs.*;
import frc.team3647subsystems.Elevator.ElevatorLevel;

public class Arm
{
	public static ArmPosition currentState;
	public static ArmPosition aimedState;

	public static WPI_TalonSRX armSRX = new WPI_TalonSRX(Constants.armSRXPin);
	public static CANSparkMax armNEO = new CANSparkMax(Constants.armNEOPin, CANSparkMaxLowLevel.MotorType.kBrushless);
	public static CANDigitalInput revNeoLimitSwitch; 
	public static CANDigitalInput fwdNeoLimitSwitch;

	public static int armEncoderCCL, armEncoderValue, armEncoderVelocity;

	public static double overrideValue;
    public static boolean manualOverride;
	
    public static void armInitialization()
    {		
		//Encoder for arm motor
		armSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs);
		armSRX.setSensorPhase(true);

		revNeoLimitSwitch = armNEO.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
		fwdNeoLimitSwitch = armNEO.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);

		// PID for motors
		armSRX.selectProfileSlot(Constants.armPID, 0);
		armSRX.config_kP(Constants.armPID, Constants.armkP, Constants.kTimeoutMs);
		armSRX.config_kI(Constants.armPID, Constants.armkI, Constants.kTimeoutMs);
		armSRX.config_kD(Constants.armPID, Constants.armkD, Constants.kTimeoutMs);
		armSRX.config_kF(Constants.armPID, Constants.armkF, Constants.kTimeoutMs);
		armSRX.config_IntegralZone(Constants.armPID, Constants.armIZone, Constants.kTimeoutMs);
		
		//arm NEO Follower Code, acrual motor that follows the SRX controller
		armNEO.follow(CANSparkMax.ExternalFollower.kFollowerPhoenix, 18);
	}

	public void configurePIDFMM(double p, double i, double d, double f, int vel, int accel)
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
		MANUAL
	}

	public static void setManualContrller(Joysticks controller)
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
		 else if(controller.dPadLeft)
			aimedState = ArmPosition.CARGOL3BACK;
		 else if(controller.dPadRight)
			aimedState = ArmPosition.CARGOL3FRONT;
	}


	public static void runArm()
	{
		setArmEncoder();

 		if(getFwdLimitSwitch())
			stopArm();
		if(getRevLimitSwitch())
			stopArm();

		switch(aimedState)
		{
			case MANUAL:
				if(!manualOverride)
				{
					overrideValue = 0;
				}
				moveManual(overrideValue);
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
				moveToHatchHanoff();
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
			default:
				break;
		}
	}

	public static void setManualOverride(double jValue)
	{
		if(Math.abs(jValue) > .05) //deadzone
		{
			manualOverride = true;
            overrideValue = jValue;
		} 
		else 
		{
			manualOverride = false;
		}
	}

	public static void moveToFwdLimitSwitch()
	{
		if(!getFwdLimitSwitch())
		{
			// rotates arm forwards
			setOpenLoop(.25);
		}
		else
		{
			setEncoderValue(Constants.armSRXFwdLimitSwitch);
			setPosition(Constants.armSRXFwdLimitSwitch);
		}
	}

	public static void moveToRevLimitSwitch()
	{
		if(!getRevLimitSwitch())
		{
			setOpenLoop(-.25);
		}
		else
		{
			resetArmEncoder();
			setPosition(0);
		}
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

	public static void moveToHatchHanoff()
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

	public static int encoderState, manualAdjustment, manualEncoderValue;

    public static void moveManual(double jValue)
	{
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
	}

	public static void setPosition(int position)
	{
		armSRX.set(ControlMode.MotionMagic, position);
	}

	public static boolean positionThreshold(double constant)
	{
		if((constant + Constants.kArmSRXPositionThreshold) > armEncoderValue && (constant - Constants.kElevatorPositionThreshold) < armEncoderValue)
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

	public static boolean stateDetection(ArmPosition position)
	{
		switch(position)
		{
			case MANUAL:
				return manualOverride;
			case FWDLIMITSWITCH:
				return getFwdLimitSwitch();
			case REVLIMITSWITCH:
				return getRevLimitSwitch();
			case FLATFORWARDS:
				return positionThreshold(Constants.armSRXFlatForwards);
			case FLATBACKWARDS:
				return positionThreshold(Constants.armSRXFlatBackwards);
			case CARGOL3FRONT:
				return positionThreshold(Constants.armSRXCargoL3Front);
			case CARGOL3BACK:
				return positionThreshold(Constants.armSRXCargoL3Back);
			case HATCHHANDOFF:
				return positionThreshold(Constants.armSRXHatchHandoff);
			case STOWED:
				return positionThreshold(Constants.armSRXStowed);
			case VERTICALSTOWED:
				return positionThreshold(Constants.armSRXVerticalStowed);
			case CARGOHANDOFF:
				return positionThreshold(Constants.armSRXCargoHandoff);
			default:
				return false;
		}
	}

	public static void setArmEncoder()
	{
		if(getRevLimitSwitch())
		{
			resetArmEncoder();
		}
		else if(getFwdLimitSwitch())
		{
			setEncoderValue(Constants.armSRXFwdLimitSwitch);
		}
		armEncoderValue = armSRX.getSelectedSensorPosition(0);
		armEncoderVelocity = armSRX.getSelectedSensorVelocity(0);
		armEncoderCCL = armSRX.getClosedLoopError(0);
	}

	public static boolean getRevLimitSwitch()
	{
		return revNeoLimitSwitch.get();
		//return armSRX.getSensorCollection().isRevLimitSwitchClosed();
	}

	public static boolean getFwdLimitSwitch()
	{
		return fwdNeoLimitSwitch.get();
		//return armSRX.getSensorCollection().isRevLimitSwitchClosed();
	}

	public static void setEncoderValue(int encoderValue)
	{
		armSRX.setSelectedSensorPosition(encoderValue, 0, Constants.kTimeoutMs);
	}

	public static void resetArmEncoder()
	{
		armSRX.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
	}
	
	public static void stopArm()
	{
		armSRX.stopMotor();
	}
	
	public static void testArmEncoders()
    {
        System.out.println("Arm Encoder Value: " + armEncoderValue + "Arm Velocity: " + armEncoderVelocity);
	}

	public static void testArmLimitSwitches()
	{
		System.out.println("Rev Limit Switch: " + getRevLimitSwitch() + "Fwd Limit Swtich: " + getFwdLimitSwitch());
	}

    public static void printArmCurrent()
    {
        System.out.println("ArmCurrent:" + armNEO.getOutputCurrent());
	}
}
