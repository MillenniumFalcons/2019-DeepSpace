//Class not created by Kunal Singla

package frc.team3647subsystems;

import frc.robot.*;
import frc.team3647inputs.Joysticks;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

public class HatchIntake 
{
	public static WristPosition currentState, aimedState;
	
	public static Solenoid clampSolenoid = new Solenoid(Constants.hatchIntakeSolinoidPin);
	public static WPI_TalonSRX wristMotor = new WPI_TalonSRX(Constants.hatchMotorPin);
	public static DigitalInput limitSwitch = new DigitalInput(Constants.hatchLimitSwitchPin);
	public static DigitalInput hatchBeamBreak = new DigitalInput(Constants.hatchIntakeBeamBreakPin);

	public static int wristEncoderCCL, wristEncoderValue, wristEncoderVelocity;
	public static double overrideValue;
	public static boolean manualOverride;


	public static void hatchIntakeInitialization()
	{
		//Motor & Sensor Direction
		wristMotor.setInverted(false);
		wristMotor.setSensorPhase(true);
		
		//Current Limiting
		wristMotor.configPeakCurrentLimit(Constants.kHatchWristPeakCurrent, Constants.kTimeoutMs);
		wristMotor.configPeakCurrentDuration(Constants.kHatchWristPeakCurrentDuration);
		wristMotor.configContinuousCurrentLimit(Constants.kHatchWristContinuousCurrent, Constants.kTimeoutMs);
		wristMotor.enableCurrentLimit(true);
		//PID
		wristMotor.config_kP(Constants.kHatchWristPID, Constants.kHatchWristP, Constants.kTimeoutMs);
		wristMotor.config_kI(Constants.kHatchWristPID, Constants.kHatchWristI, Constants.kTimeoutMs);
		wristMotor.config_kD(Constants.kHatchWristPID, Constants.kHatchWristD, Constants.kTimeoutMs);
		wristMotor.config_kF(Constants.kHatchWristPID, Constants.kHatchWristF, Constants.kTimeoutMs);

		//Motion Magic Constants
		wristMotor.configMotionAcceleration(Constants.kHatchWristAcceleration, Constants.kTimeoutMs);
		wristMotor.configMotionCruiseVelocity(Constants.kHatchWristCruiseVelocity, Constants.kTimeoutMs);

		aimedState = null;
		resetEncoder();
	}

	public void configPIDFVA(double p, double i, double d, double f, int vel, int accel) //for configuring PID values
	{
		//PID
		wristMotor.config_kP(Constants.kHatchWristPID, p, Constants.kTimeoutMs);
		wristMotor.config_kI(Constants.kHatchWristPID, i, Constants.kTimeoutMs);
		wristMotor.config_kD(Constants.kHatchWristPID, d, Constants.kTimeoutMs);
		wristMotor.config_kF(Constants.kHatchWristPID, f, Constants.kTimeoutMs);
		
		// motion magic
		wristMotor.configMotionAcceleration(accel, Constants.kTimeoutMs);
		wristMotor.configMotionCruiseVelocity(vel, Constants.kTimeoutMs);

	}

	public static enum WristPosition
	{
		STOWED,
		GROUND,
		SCORE,
		HANDOFF,
		MANUAL
	}
	
	public static void runHatchClamp(boolean joyvalue) 
	{
		if(joyvalue)
		{
			if(!clampSolenoid.get())
			{
				openClamp();
				Timer.delay(.1);
			}
			else
			{
				closeClamp();
				Timer.delay(.1);
			}
		}
    }
	public static void openClamp()
	{
		clampSolenoid.set(true);
	}
	
	public static void closeClamp()
	{
		clampSolenoid.set(false);
	}

	public static void setHatchIntakeWristController(Joysticks controller)
	{
		if(manualOverride)
			aimedState = WristPosition.MANUAL;
		else if(controller.leftJoyStickX > .5)
			aimedState = WristPosition.SCORE;
		else if(controller.leftJoyStickX < -.5)
			aimedState = WristPosition.HANDOFF;
		else if(controller.leftJoyStickY > .5)
			aimedState = WristPosition.STOWED;
		else if(controller.leftJoyStickY < -.5)
			aimedState = WristPosition.GROUND;

		if(controller.leftBumper && clampSolenoid.get())
			closeClamp();
		else if(controller.leftBumper &&  !clampSolenoid.get())
			openClamp();
	}
	public static void runHatchIntakeWrist()
	{
		//setWristEncoder();
		//setManualOverride(controller.leftJoyStickY);
		//get joy input

			
		//move to positions
		if(aimedState != null)
		{
			System.out.println("Moving hatch to " + aimedState);
			switch(aimedState)
			{
				// case MANUAL:
				// 	if(!manualOverride)
				// 	{
				// 		overrideValue = 0;
				// 	}
				// 	moveManual(overrideValue);
				// 	break;
				case STOWED:
					//closeClamp();
					moveToStowed();
					break;
				case SCORE:
					//closeClamp();
					moveToScore();
					break;
				case GROUND:
					moveToGround();
					//openClamp();
					break;
				case HANDOFF:
					moveToHandoff();
					break;
				default:
					break;
			}
		}
	}	

	public static void moveToStowed()
	{
		if(getLimitSwitch())
		{
			System.out.println("Hatch Intake Stowed");
			stopWrist();
			resetEncoder();
		}
		else
		{
			setOpenLoop(-0.25);
			//setPosition(0);
		}
	}

	//Motion Magic movement-------------------------------------
	public static void moveToGround()
	{
		setPosition(Constants.hatchIntakeGround);
	}

	public static void moveToScore()
	{
		setPosition(Constants.hatchIntakeScore);
	}

	public static void moveToHandoff()
	{
		System.out.println("Moving wrist to hatchhandoff");
		setPosition(Constants.hatchIntakeHandoff);
	}

	public static void setPosition(int position)
	{
		wristMotor.set(ControlMode.MotionMagic, position);
	}
	//----------------------------------------------------------

	//Manual movement-------------------------------------------
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

	private static int encoderState, manualAdjustment, manualEncoderValue;

	public static void moveManual(double jValue)
	{
		if(jValue > 0)
		{
			setOpenLoop(jValue * 0.5);
			manualAdjustment = 50;
			encoderState = 0;
		}
		else if(jValue < 0)
		{
			setOpenLoop(jValue * 0.5);
			manualAdjustment = 0;
			encoderState = 0;
		}
		else
		{
			switch(encoderState)
			{
				case 0:
					manualEncoderValue = wristEncoderValue + manualAdjustment;
					encoderState = 1;
					break;
				case 1:
					setPosition(manualEncoderValue);
					break;
			}
		}
	}
	
	private static void setOpenLoop(double power)
	{
		wristMotor.set(ControlMode.PercentOutput, power);

	}
	//---------------------------------------------------------

	public static boolean positionThreshold(double constant)
	{
		if((constant + Constants.kWristPositionThreshold) > wristEncoderValue && (constant - Constants.kWristPositionThreshold) < wristEncoderValue)
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	// Encoder methods------------------------------------------
	public static void setWristEncoder()
	{
		if(getLimitSwitch())
		{
			resetEncoder();
		}
		wristEncoderValue = wristMotor.getSelectedSensorPosition(0);
		wristEncoderVelocity = wristMotor.getSelectedSensorVelocity(0);
		wristEncoderCCL = wristMotor.getClosedLoopError(0);
	}

	public static void setEncoderValue(int encoderValue)
	{
		wristMotor.setSelectedSensorPosition(encoderValue, 0, Constants.kTimeoutMs);
	}
	//----------------------------------------------------------

	public static void resetEncoder()
	{
		wristMotor.setSelectedSensorPosition(0, Constants.kHatchWristPID, Constants.kTimeoutMs);
	}

	public static boolean getLimitSwitch()
	{
		return !limitSwitch.get();
	}

	public static boolean detectHatch()
	{
		return hatchBeamBreak.get();
	}



	public static void printPosition()
	{
		System.out.println("Encoder Value: " + wristMotor.getSelectedSensorPosition(0));
	}



	public static void stopWrist()
	{
		wristMotor.stopMotor();
	}

	public static WristPosition getCurrentState()
	{
		return currentState;
	}

	public static WristPosition getAimedState()
	{
		return aimedState;
	}
}
