package frc.team3647pistons;

import frc.robot.*;
import frc.team3647inputs.Joysticks;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;

public class IntakeHatch 
{
	private HatchPosition currentState;
	private HatchPosition aimedState;
	private Solenoid piston = new Solenoid(Constants.hatchIntakeFC);
	private WPI_TalonSRX hatchSRX = new WPI_TalonSRX(Constants.hatchMotorPin);
	private DigitalInput limitSwitchIntake = new DigitalInput(Constants.hatchLimitSwitchPin);

	private int encoderValue = getSelectedSensorPosition();
	private int prevEncoderValue;


	public IntakeHatch()
	{
		//Motor Direction
		this.hatchSRX.set(ControlMode.PercentOutput, 0);
		this.hatchSRX.setInverted(true);
		this.hatchSRX.setSensorPhase(true);
		//Current Limiting
		this.hatchSRX.configPeakCurrentLimit(Constants.peakCurrentHatch, Constants.kTimeoutMs);
		this.hatchSRX.configContinuousCurrentLimit(Constants.continuousCurrentHatch, Constants.kTimeoutMs);
		this.hatchSRX.enableCurrentLimit(true);
		//PID
		this.hatchSRX.config_kP(Constants.hatchIntakePIDIdx, Constants.kPHatch);
		this.hatchSRX.config_kI(Constants.hatchIntakePIDIdx, Constants.kIHatch);
		this.hatchSRX.config_kD(Constants.hatchIntakePIDIdx, Constants.kDHatch);
		this.hatchSRX.config_kF(Constants.hatchIntakePIDIdx, Constants.kFHatch);
		this.aimedState = HatchPosition.INSIDE;

		// motion magic
		this.hatchSRX.configMotionAcceleration(Constants.kHatchAcceleration, Constants.kTimeoutMs);
		this.hatchSRX.configMotionCruiseVelocity(Constants.kHatchVelocity, Constants.kTimeoutMs);
	}

	public void configPIDFMM(double p, double i, double d, double f, int vel, int accel)
	{
				//PID
				this.hatchSRX.config_kP(Constants.hatchIntakePIDIdx, p);
				this.hatchSRX.config_kI(Constants.hatchIntakePIDIdx, i);
				this.hatchSRX.config_kD(Constants.hatchIntakePIDIdx, d);
				this.hatchSRX.config_kF(Constants.hatchIntakePIDIdx, f);
		
				// motion magic
				this.hatchSRX.configMotionAcceleration(accel, Constants.kTimeoutMs);
				this.hatchSRX.configMotionCruiseVelocity(vel, Constants.kTimeoutMs);

	}

	public static enum HatchPosition
	{
		INSIDE,
		OUTSIDE,
		LOADING
	}
	
	public void openIntake()
	{
		piston.set(true);
	}
	
	public void closeIntake()
	{
		piston.set(false);
	}

	/**
	 * 
	 * @param positionInput 1 = inside drivetrain (0); 2 = loading position for arm (90); 3 = outside intake position (180)
	 */
	public void setPosition(HatchPosition positionInput)
	{
		switch(positionInput)
		{
			case INSIDE:
				this.resetPosition();
				if(this.stateRecognizer(positionInput))
					currentState = HatchPosition.INSIDE;
				break;
			case LOADING:
				this.setMMPosition(Constants.hatchIntakeLoad);
				if(this.stateRecognizer(positionInput))
					this.currentState = HatchPosition.LOADING;
				break;
			case OUTSIDE:
				this.setMMPosition(Constants.hatchIntakeOutside);
				if(this.stateRecognizer(positionInput))
					this.currentState = HatchPosition.OUTSIDE;
				break;
			default:
				System.out.println("INVALID HATCH INTAKE POSITION INPUT");
				break;
		}
		// if(positionInput == HatchPosition.INSIDE)
		// {
		// 	resetPosition();
		// 	if(stateRecognizer(HatchPosition.INSIDE))
		// 		currentState = HatchPosition.INSIDE;
		// 	//hatchSRX.setSelectedSensorPosition(0,0,Constants.kTimeoutMs);
		// }
		// else if(positionInput == HatchPosition.LOADING)
		// {
		// 	setMMPosition(Constants.hatchIntakeLoad);
		// 	if(stateRecognizer(HatchPosition.LOADING))
		// 			currentState = HatchPosition.LOADING;
		// }			
		// else if(positionInput == HatchPosition.OUTSIDE)
		// {
		// 	setMMPosition(Constants.hatchIntakeOutside);
		// 	if(stateRecognizer(HatchPosition.OUTSIDE))
		// 			currentState = HatchPosition.OUTSIDE;
		// }
		// else
		// 	System.out.println("INVALID HATCH INTAKE POSITION INPUT");
	}

	private void resetPosition()
	{
		if(getLimitSwitch() == true)
		{
			this.stopMotor();
			this.setEncoderPosition();
			// System.out.println("RESET POSITION");
		}
		else
		{
			this.moveMotor(.25);
		}
	}
	
	private void moveMotor(double power)
	{
		hatchSRX.set(ControlMode.PercentOutput, power);

	}

	public void setMMPosition(int encoderInput) //MM = Motion Magic
	{
		hatchSRX.set(ControlMode.MotionMagic, encoderInput);
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

	private boolean stateRecognizer(HatchPosition level)
	{
		switch(level)
		{
			case LOADING:
				if(this.stateThreshold(Constants.hatchIntakeLoad, encoderValue, 25))
					return true;
				else
					return false;
			case OUTSIDE:
				if(this.stateThreshold(Constants.hatchIntakeOutside, encoderValue, 25))
					return true;
				else
					return false;
			case INSIDE:
				return this.getLimitSwitch();
			default:
				return false;
		}

	}
	
	public void runIntake(Joysticks controller)
	{
		//Assigns aimed state based on controller input
		if(controller.dPadUp)
			this.aimedState = HatchPosition.LOADING;
		else if(controller.dPadDown)
			this.aimedState = HatchPosition.INSIDE;
		else if(controller.dPadSide)
			this.aimedState = HatchPosition.OUTSIDE;

		//updates position based on aimedstate
		switch(aimedState)
		{
			case INSIDE:
				this.setPosition(HatchPosition.INSIDE);
				break;
			case LOADING:
				this.setPosition(HatchPosition.LOADING);
				break;
			case OUTSIDE:
				this.setPosition(HatchPosition.OUTSIDE);
				break;
			default:
				break;
		}

		if(controller.leftBumper)
		{
			this.openIntake();
		}
		else
		{
			this.closeIntake();
		}
	}

	private void updateEncoder()
	{
		this.prevEncoderValue = this.encoderValue;
		this.encoderValue = this.getSelectedSensorPosition();
	}

	public int getSelectedSensorPosition()
	{
		return this.hatchSRX.getSelectedSensorPosition(Constants.hatchIntakePIDIdx);
	}

	public void setEncoderPosition()
	{
		hatchSRX.setSelectedSensorPosition(0);
	}

	public void resetEncoder()
	{
		hatchSRX.setSelectedSensorPosition(0, Constants.hatchIntakePIDIdx, Constants.kTimeoutMs);
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

	public boolean getLimitSwitch()
	{
		return !limitSwitchIntake.get();
	}

	public void stopMotor()
	{
		hatchSRX.stopMotor();
	}

	public HatchPosition getCurrentState()
	{
		return this.currentState;
	}

	public HatchPosition getAimedState()
	{
		return this.aimedState;
	}
}
