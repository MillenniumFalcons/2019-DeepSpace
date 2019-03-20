//Class not created by Kunal Singla

package frc.team3647subsystems;

import frc.robot.*;
import frc.team3647inputs.Joysticks;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;


public class Elevator
{

	public static ElevatorLevel currentState, lastState, aimedState;
	
	// Sensor at bottom of elevator
	public static DigitalInput limitSwitch = new DigitalInput(Constants.elevatorBeamBreakPin);

	// Elevator motors
    public static WPI_TalonSRX elevatorMaster = new WPI_TalonSRX(Constants.ElevatorGearboxSRX); //8
	public static VictorSPX GearboxSPX1 = new VictorSPX(Constants.ElevatorGearboxSPX1);	//12
	public static VictorSPX GearboxSPX2 = new VictorSPX(Constants.ElevatorGearboxSPX2); //13
	public static VictorSPX GearBoxSPX3 = new VictorSPX(14);

	public static int elevatorEncoderCCL, elevatorEncoderValue, elevatorEncoderVelocity;
	
	private static double overrideValue;
	private static boolean manualOverride;
    
    public static void elevatorInitialization()
	{
		currentState = null;
		aimedState = null;
		lastState = null;
		//Config Sensors for encoder
		elevatorMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs);
		elevatorMaster.setSensorPhase(true);

		// Configure PID Values
		elevatorMaster.selectProfileSlot(Constants.interstageSlotIdx, 0);

		elevatorMaster.config_kP(Constants.interstageSlotIdx, Constants.interstagePIDF[0], Constants.kTimeoutMs);
		elevatorMaster.config_kI(Constants.interstageSlotIdx, Constants.interstagePIDF[1], Constants.kTimeoutMs);
		elevatorMaster.config_kD(Constants.interstageSlotIdx, Constants.interstagePIDF[2], Constants.kTimeoutMs);
		elevatorMaster.config_kF(Constants.interstageSlotIdx, Constants.interstagePIDF[3], Constants.kTimeoutMs);

		// GearboxMaster.config_IntegralZone(Constants.carriagePIDX, Constants.carriageIZone, Constants.kTimeoutMs);

		// //Motion Magic Constants
		elevatorMaster.configMotionCruiseVelocity(Constants.kElevatorCruiseVelocity, Constants.kTimeoutMs);
		elevatorMaster.configMotionAcceleration(Constants.kElevatorAcceleration, Constants.kTimeoutMs);

		GearboxSPX2.follow(elevatorMaster);
		GearboxSPX1.follow(elevatorMaster);
		GearBoxSPX3.follow(elevatorMaster);
		GearboxSPX2.setInverted(false);
		elevatorMaster.setInverted(false);
		GearboxSPX1.setInverted(false);

		Elevator.elevatorMaster.enableCurrentLimit(true);
		Elevator.elevatorMaster.configContinuousCurrentLimit(35);

		elevatorMaster.setNeutralMode(NeutralMode.Brake);
		setEncoderValue(Constants.elevatorStartingStowed);
	}
	
	public static void elevatorTeleopInit()
	{
			//Config Sensors for encoder
			elevatorMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs);
			elevatorMaster.setSensorPhase(true);
	
			// Configure PID Values
			elevatorMaster.selectProfileSlot(Constants.interstageSlotIdx, 0);
	
			elevatorMaster.config_kP(Constants.interstageSlotIdx, Constants.interstagePIDF[0], Constants.kTimeoutMs);		
			elevatorMaster.config_kI(Constants.interstageSlotIdx, Constants.interstagePIDF[1], Constants.kTimeoutMs);	
			elevatorMaster.config_kD(Constants.interstageSlotIdx, Constants.interstagePIDF[2], Constants.kTimeoutMs);
			elevatorMaster.config_kF(Constants.interstageSlotIdx, Constants.interstagePIDF[3], Constants.kTimeoutMs);
			
			// GearboxMaster.config_IntegralZone(Constants.carriagePIDX, Constants.carriageIZone, Constants.kTimeoutMs);
	
			// //Motion Magic Constants
			elevatorMaster.configMotionCruiseVelocity(Constants.kElevatorCruiseVelocity, Constants.kTimeoutMs);
			elevatorMaster.configMotionAcceleration(Constants.kElevatorAcceleration, Constants.kTimeoutMs);
	
			GearboxSPX2.follow(elevatorMaster);
			GearboxSPX1.follow(elevatorMaster);
			GearBoxSPX3.follow(elevatorMaster);
			GearboxSPX2.setInverted(false);
			elevatorMaster.setInverted(false);
			GearboxSPX1.setInverted(false);
	
			Elevator.elevatorMaster.enableCurrentLimit(true);
			Elevator.elevatorMaster.configContinuousCurrentLimit(35);
	
			elevatorMaster.setNeutralMode(NeutralMode.Brake);
	}

	public static void configurePIDFMM(double p, double i, double d, double f, int vel, int accel)
	{
		elevatorMaster.selectProfileSlot(Constants.interstageSlotIdx, 0);

		elevatorMaster.config_kP(Constants.interstageSlotIdx, p, Constants.kTimeoutMs);		
		elevatorMaster.config_kI(Constants.interstageSlotIdx, i, Constants.kTimeoutMs);	
		elevatorMaster.config_kD(Constants.interstageSlotIdx, d, Constants.kTimeoutMs);
		elevatorMaster.config_kF(Constants.interstageSlotIdx, f, Constants.kTimeoutMs);
		//Motion Magic Constants
		elevatorMaster.configMotionCruiseVelocity(vel, Constants.kTimeoutMs);
        elevatorMaster.configMotionAcceleration(accel, Constants.kTimeoutMs);
	}

	public enum ElevatorLevel
	{
		MANUAL(-1),
		STOP(-1),
		BOTTOM(0),
        CARGOHANDOFF(Constants.elevatorCargoHandoff),
        HATCHHANDOFF(Constants.elevatorHatchHandoff),
		HATCHL2(Constants.elevatorHatchL2),
		HATCHL3(Constants.elevatorHatchL3), //also cargo lvl3
		CARGOL2(Constants.elevatorCargoL2),
		CARGO1(Constants.elevatorCargoL1),
        CARGOSHIP(Constants.elevatorCargoShip),
		STOWED(Constants.elevatorStowed),
		MINROTATE(Constants.elevatorMinRotation),
		VERTICALSTOWED(Constants.elevatorMinRotation),
		START(-1);

		public int encoderVal;

		ElevatorLevel(int encoderVal)
		{
			this.encoderVal = encoderVal;
		}
	}

	public static void setManualController(Joysticks controller)
	{
			if(controller.buttonA)
				aimedState = ElevatorLevel.BOTTOM;     //if hatch
			else if(controller.buttonB)
				aimedState = ElevatorLevel.HATCHL2;
			else if(controller.buttonY)
				aimedState = ElevatorLevel.HATCHL3;
			else if(controller.buttonX)
				aimedState = ElevatorLevel.MINROTATE;
	}

	public static void setElevatorManualControl(Joysticks controller)
	{
		setManualOverride(controller.leftJoyStickY);
	}

	public static void runElevator()
	{
		setElevatorEncoder();
		updateLivePosition();
		if(aimedState != null)
		{
			switch(aimedState) //check if aimed state has a value
			{
				case MANUAL:
					// if(!manualOverride)
					// 	overrideValue = 0;
					// moveManual(overrideValue);
					break;
				case STOP:
					stopElevator();
					break;
				case START:
					moveToBottomStart();
					break;
				case BOTTOM:
					moveToBottom();
					break;
				default:
					if(aimedState.encoderVal != -1)
						setPosition(aimedState.encoderVal);
					break;
			}
		}
	}
	
	

	// Motion Magic based movement------------------------------
	public static void setPosition(int position)
	{
		elevatorMaster.set(ControlMode.MotionMagic, position);
	}
	static void moveToBottomStart()
	{
		if(getLimitSwitch())
		{
			stopElevator();
			resetElevatorEncoder();
		}
		else
			setOpenLoop(-.3);
	}
	
	private static void moveToBottom()
	{
		if(positionThreshold(0) && !getLimitSwitch())
			moveToBottomStart();
		else
			setPosition(0);
		
	}
    
	//----------------------------------------------------------

	//Manual movement-------------------------------------------
	public static void setManualOverride(double jValue)
	{
		if(Math.abs(jValue) > .15) //deadzone
		{
			manualOverride = true;
			aimedState = ElevatorLevel.MANUAL;
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
		setElevatorEncoder();
		updateLivePosition();
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
					manualEncoderValue = elevatorEncoderValue + manualAdjustment;
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
		elevatorMaster.set(ControlMode.PercentOutput, power);
	}
	//----------------------------------------------------------

	private static boolean positionThreshold(double constant)
	{
		return (constant + Constants.kElevatorPositionThreshold) > elevatorEncoderValue && 
				(constant - Constants.kElevatorPositionThreshold) < elevatorEncoderValue;
	}

	private static void updateLivePosition()
	{
		if(getLimitSwitch())
			currentState = ElevatorLevel.BOTTOM;
		else if(positionThreshold(Constants.elevatorCargoHandoff))
			currentState = ElevatorLevel.CARGOHANDOFF;
		else if(positionThreshold(Constants.elevatorHatchHandoff))
			currentState = ElevatorLevel.HATCHHANDOFF;
		else if(positionThreshold(Constants.elevatorHatchL2))
			currentState = ElevatorLevel.HATCHL2;
		else if(positionThreshold(Constants.elevatorHatchL3))
			currentState = ElevatorLevel.HATCHL3;
		else if(positionThreshold(Constants.elevatorCargoL2))
			currentState = ElevatorLevel.CARGOL2;
		else if(positionThreshold(Constants.elevatorCargoShip))
			currentState = ElevatorLevel.CARGOSHIP;
		else if(positionThreshold(Constants.elevatorStowed) || positionThreshold(Constants.elevatorStartingStowed))
			currentState = ElevatorLevel.STOWED;
		else if(positionThreshold(Constants.elevatorMinRotation))
			currentState = ElevatorLevel.MINROTATE;
		else if(positionThreshold(Constants.elevatorVerticalStowed))
			currentState = ElevatorLevel.VERTICALSTOWED;
		else
			currentState = ElevatorLevel.MANUAL;
	}

	//Encoder Methods-------------------------------------------
	public static void setElevatorEncoder()
	{
		if(getLimitSwitch())
			resetElevatorEncoder();
		elevatorEncoderValue = elevatorMaster.getSelectedSensorPosition(0);
		elevatorEncoderVelocity = elevatorMaster.getSelectedSensorVelocity(0);
		elevatorEncoderCCL = elevatorMaster.getClosedLoopError(0);
	}

	private static void setEncoderValue(int encoderValue)
	{
		elevatorMaster.setSelectedSensorPosition(encoderValue, 0, Constants.kTimeoutMs);
	}
	
	private static void resetElevatorEncoder()
	{
		setEncoderValue(0);
	}

	public static int getStateEncoder(ElevatorLevel futureAimedState)
	{
		if(futureAimedState != null && futureAimedState.encoderVal != -1)
			return futureAimedState.encoderVal;
		return -1;
	}
	//----------------------------------------------------------

	private static boolean getLimitSwitch()
	{
		return limitSwitch.get();
	}
	
	public static void stopElevator()
	{
		elevatorMaster.stopMotor();
	}

	public static boolean isAboveMinRotate(int threshold)
	{
		return (elevatorEncoderValue >= Constants.elevatorMinRotation + threshold);
	}
	
	public static void printElevatorEncoders()
    {
        System.out.println("Elevator Encoder Value: " + elevatorEncoderValue );//+ "\nElevator Velocity: " + elevatorEncoderVelocity);
	}

	public static void printBannerSensor()
    {
    	System.out.println("Banner Sensor Value: " + getLimitSwitch());
    }

    public static void printElevatorCurrent()
    {
        System.out.println("Elevator Current: " + elevatorMaster.getOutputCurrent());
	}

	public static void disableElevator()
	{
		aimedState = null;
		elevatorMaster.setNeutralMode(NeutralMode.Coast);
		stopElevator();
	}
}