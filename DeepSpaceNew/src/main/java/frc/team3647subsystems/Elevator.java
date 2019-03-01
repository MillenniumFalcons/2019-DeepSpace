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
    public static VictorSPX GearboxSPX2= new VictorSPX(Constants.ElevatorGearboxSPX2); //13

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
		GearboxSPX2.setInverted(false);
		elevatorMaster.setInverted(false);
		GearboxSPX1.setInverted(false);

		Elevator.elevatorMaster.enableCurrentLimit(true);
		Elevator.elevatorMaster.configContinuousCurrentLimit(50);

		elevatorMaster.setNeutralMode(NeutralMode.Brake);
		setEncoderValue(Constants.elevatorStartingStowed);
	}

	public static void configurePIDFMM(double p, double i, double d, double f, int vel, int accel)
	{
		elevatorMaster.selectProfileSlot(Constants.interstageSlotIdx, 0);

		elevatorMaster.config_kP(Constants.interstageSlotIdx, p, Constants.kTimeoutMs);		
		elevatorMaster.config_kI(Constants.interstageSlotIdx, i, Constants.kTimeoutMs);	
		elevatorMaster.config_kD(Constants.interstageSlotIdx, d, Constants.kTimeoutMs);
		elevatorMaster.config_kF(Constants.interstageSlotIdx, f, Constants.kTimeoutMs);
		
		// GearboxMaster.config_IntegralZone(Constants.carriagePIDIdx, Constants.carriageIZone, Constants.kTimeoutMs);
		
		//Motion Magic Constants
		elevatorMaster.configMotionCruiseVelocity(vel, Constants.kTimeoutMs);
        elevatorMaster.configMotionAcceleration(accel, Constants.kTimeoutMs);
	}

	public enum ElevatorLevel
	{
		MANUAL,
		STOP,
		BOTTOM,
        CARGOHANDOFF,
        HATCHHANDOFF,
		HATCHL2,
		HATCHL3, //also cargo lvl3
		CARGOL2,
		CARGO1,
        CARGOSHIP,
		STOWED,
		MINROTATE,
		VERTICALSTOWED,
		START
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

	public static void runElevator()
	{
		System.out.println("Elevator aimedPos " + aimedState);
		setElevatorEncoder();
		updateLivePosition();
		// setManualOverride(controller.leftJoyStickY);   
		if(aimedState != null)
		{
			//System.out.println("Elevator moving to: " + aimedState);
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
				case BOTTOM:
					moveToBottom();
					break;
				case START:
					moveToBottomStart();
					break;
				case CARGOHANDOFF:
					moveToCargoHandoff();
					break;
				case HATCHHANDOFF:
					moveToHatchHandoff();
					break;
				case HATCHL2:
					moveToHatchL2();
					break;
				case HATCHL3:
					moveToHatchL3();
					break;
				case CARGO1:
					moveToCargoL1();
					break;
				case CARGOL2:
					moveToCargoL2();
					break;
				case CARGOSHIP:
					moveToCargoShip();
					break;
				case STOWED:
					moveToStowed();
					break;
				case VERTICALSTOWED:
					moveToVerticalStowed();
					break;
				case MINROTATE:
					moveToMinRotate();
					break;
				default:
					System.out.println("Unreachable ELEVATOR Position");
					break;
			}
		}
	}
	
	

	// Motion Magic based movement------------------------------
	public static void setPosition(int position)
	{
		elevatorMaster.set(ControlMode.MotionMagic, position);
	}
	private static void moveToBottomStart()
	{
		if(getLimitSwitch())
		{
			stopElevator();
			resetElevatorEncoder();
		}
		else
		{
			//setPosition(0);
			setOpenLoop(-.15);
		}
	}
	
	private static void moveToBottom()
	{
		setPosition(0);
	}
    
    private static void moveToCargoHandoff()
    {
		setPosition(Constants.elevatorCargoHandoff);
    }
    
    private static void moveToHatchHandoff()
    {
        setPosition(Constants.elevatorHatchHandoff);
    }

    private static void moveToHatchL2()
    {
        setPosition(Constants.elevatorHatchL2);
    }

    private static void moveToHatchL3()
    {
        setPosition(Constants.elevatorHatchL3);
    }
	private static void moveToCargoL1() 
	{
		setPosition(Constants.elevatorCargoL1);
	}
    private static void moveToCargoL2()
    {
        setPosition(Constants.elevatorCargoL2);
    }

    private static void moveToCargoShip()
    {
        setPosition(Constants.elevatorCargoShip);
    }

    private static void moveToStowed()
    {
        setPosition(Constants.elevatorStowed);
	}
	
	private static void moveToVerticalStowed()
	{
		setPosition(Constants.elevatorVerticalStowed);
	}

	private static void moveToMinRotate()
	{
		setPosition(Constants.elevatorMinRotation);
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
		// Percent Output		
		elevatorMaster.set(ControlMode.PercentOutput, power);

	}
	//----------------------------------------------------------

	private static boolean positionThreshold(double constant)
	{
		if((constant + Constants.kElevatorPositionThreshold) > elevatorEncoderValue && (constant - Constants.kElevatorPositionThreshold) < elevatorEncoderValue)
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	private static void updateLivePosition()
	{
		if(getLimitSwitch())
		{
			currentState = ElevatorLevel.BOTTOM;
			lastState = ElevatorLevel.BOTTOM;
		}
		else if(positionThreshold(Constants.elevatorCargoHandoff))
		{
			currentState = ElevatorLevel.CARGOHANDOFF;
			lastState = ElevatorLevel.CARGOHANDOFF;
		}
		else if(positionThreshold(Constants.elevatorHatchHandoff))
		{
			currentState = ElevatorLevel.HATCHHANDOFF;
			currentState = ElevatorLevel.HATCHHANDOFF;
		}
		else if(positionThreshold(Constants.elevatorHatchL2))
		{
			currentState = ElevatorLevel.HATCHL2;
			lastState = ElevatorLevel.HATCHL2;
		}
		else if(positionThreshold(Constants.elevatorHatchL3))
		{
			currentState = ElevatorLevel.HATCHL3;
			lastState = ElevatorLevel.HATCHL3;
		}
		else if(positionThreshold(Constants.elevatorCargoL2))
		{
			currentState = ElevatorLevel.CARGOL2;
			lastState = ElevatorLevel.CARGOL2;
		}
		else if(positionThreshold(Constants.elevatorCargoShip))
		{
			currentState = ElevatorLevel.CARGOSHIP;
			lastState = ElevatorLevel.CARGOSHIP;
		}
		else if(positionThreshold(Constants.elevatorStowed) || positionThreshold(Constants.elevatorStartingStowed))
		{
			currentState = ElevatorLevel.STOWED;
			lastState = ElevatorLevel.STOWED;
		}
		else if(positionThreshold(Constants.elevatorMinRotation))
		{
			currentState = ElevatorLevel.MINROTATE;
			lastState = ElevatorLevel.MINROTATE;
		}
		else if(positionThreshold(Constants.elevatorVerticalStowed))
		{
			currentState = ElevatorLevel.VERTICALSTOWED;
			lastState = ElevatorLevel.VERTICALSTOWED;
		}
		else
			currentState = ElevatorLevel.MANUAL;
	}

	private static boolean stateDetection(ElevatorLevel level)
	{
        switch(level)
		{
			case MANUAL:
				return manualOverride;
            case BOTTOM:
                return getLimitSwitch();
            case CARGOHANDOFF:
                return positionThreshold(Constants.elevatorCargoHandoff);
            case HATCHHANDOFF:
                return positionThreshold(Constants.elevatorHatchHandoff);
            case HATCHL2:
                return positionThreshold(Constants.elevatorHatchL2);
            case HATCHL3:
                return positionThreshold(Constants.elevatorHatchL3);
            case CARGOL2:
                return positionThreshold(Constants.elevatorHatchL3);
            case CARGOSHIP:
                return positionThreshold(Constants.elevatorCargoShip);
			case STOWED:
				return positionThreshold(Constants.elevatorStowed);
			case MINROTATE:
				return positionThreshold(Constants.elevatorMinRotation);
			case VERTICALSTOWED:
				return positionThreshold(Constants.elevatorVerticalStowed);
            default:
                return false;
        }
	}
	
	//Encoder Methods-------------------------------------------
	public static void setElevatorEncoder()
	{
		if(getLimitSwitch())
		{
			resetElevatorEncoder();
		}
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
		elevatorMaster.setSelectedSensorPosition(0,Constants.interstageSlotIdx, Constants.kTimeoutMs);
	}

	public static int getStateEncoder(ElevatorLevel futureAimedState)
	{
		if(futureAimedState != null)
		{
			switch(futureAimedState) //check if aimed state has a value
			{
				case STOP:
					return elevatorEncoderValue;
				case BOTTOM:
					return 0;
				case START:
					return 0;
				case CARGOHANDOFF:
					return Constants.elevatorCargoHandoff;
				case HATCHHANDOFF:
					return Constants.elevatorHatchHandoff;
				case HATCHL2:
					return Constants.elevatorHatchL2;
				case HATCHL3:
					return Constants.elevatorHatchL3;
				case CARGO1:
					return Constants.elevatorCargoL1;
				case CARGOL2:
					return Constants.elevatorCargoL2;
				case CARGOSHIP:
					return Constants.elevatorCargoShip;
				case STOWED:
					return Constants.elevatorStowed;
				case VERTICALSTOWED:
					return Constants.elevatorVerticalStowed;
				case MINROTATE:
					return Constants.elevatorMinRotation;
				default:
					return -1;
			}
		}
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
        System.out.println("Elevator Encoder Value: " + elevatorEncoderValue + "\nElevator Velocity: " + elevatorEncoderVelocity);
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