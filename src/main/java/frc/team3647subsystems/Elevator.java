//Class not created by Kunal Singla

package frc.team3647subsystems;

import frc.robot.*;
import frc.team3647inputs.Joysticks;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
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

	public static int elevatorEncoderCCL, elevatorEncoderValue, elevatorEncoderVelocity;
	
	public static double overrideValue;
	public static boolean manualOverride;
    
    public static void elevatorInitialization()
	{
		currentState = null;
		aimedState = null;
		lastState = null;
        //Config Sensors for encoder
        elevatorMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs);
		elevatorMaster.setSensorPhase(true);

		// Configure PID Values
		elevatorMaster.selectProfileSlot(Constants.interstagePID, 0);

		elevatorMaster.config_kP(Constants.interstagePID, Constants.interstageP, Constants.kTimeoutMs);		
		elevatorMaster.config_kI(Constants.interstagePID, Constants.interstageI, Constants.kTimeoutMs);	
		elevatorMaster.config_kD(Constants.interstagePID, Constants.interstageD, Constants.kTimeoutMs);
		elevatorMaster.config_kF(Constants.interstagePID, Constants.interstageF, Constants.kTimeoutMs);
		
		// GearboxMaster.config_IntegralZone(Constants.carriagePIDX, Constants.carriageIZone, Constants.kTimeoutMs);

		// //Motion Magic Constants
		elevatorMaster.configMotionCruiseVelocity(Constants.kElevatorCruiseVelocity, Constants.kTimeoutMs);
        elevatorMaster.configMotionAcceleration(Constants.kElevatorAcceleration, Constants.kTimeoutMs);

        GearboxSPX2.follow(elevatorMaster);
        GearboxSPX1.follow(elevatorMaster);
		GearboxSPX2.setInverted(false);
		elevatorMaster.setInverted(false);
		GearboxSPX1.setInverted(false);

		setEncoderValue(Constants.elevatorStartingStowed);
	}

	public static void configurePIDFMM(double p, double i, double d, double f, int vel, int accel)
	{
		elevatorMaster.selectProfileSlot(Constants.interstagePID, 0);

		elevatorMaster.config_kP(Constants.interstagePID, p, Constants.kTimeoutMs);		
		elevatorMaster.config_kI(Constants.interstagePID, i, Constants.kTimeoutMs);	
		elevatorMaster.config_kD(Constants.interstagePID, d, Constants.kTimeoutMs);
		elevatorMaster.config_kF(Constants.interstagePID, f, Constants.kTimeoutMs);
		
		// GearboxMaster.config_IntegralZone(Constants.carriagePIDIdx, Constants.carriageIZone, Constants.kTimeoutMs);
		
		//Motion Magic Constants
		elevatorMaster.configMotionCruiseVelocity(vel, Constants.kTimeoutMs);
        elevatorMaster.configMotionAcceleration(accel, Constants.kTimeoutMs);
	}

	public enum ElevatorLevel
	{
		MANUAL,
		BOTTOM,
        CARGOHANDOFF,
        HATCHHANDOFF,
		HATCHL2,
		HATCHL3, //also cargo lvl3
        CARGOL2,
        CARGOSHIP,
		STOWED,
		MINROTATE,
		VERTICALSTOWED
	}

	public static void setManualController(Joysticks controller)
	{
		//setManualOverride(controller.rightJoyStickY);
		// if(manualOverride)
		// 	aimedState = ElevatorLevel.MANUAL;
		// else if(BallShooter.cargoDetection())
		// {
		// 	if(controller.buttonA)
		// 		aimedState = ElevatorLevel.BOTTOM;
		// 	else if(controller.buttonB)
		// 		aimedState = ElevatorLevel.CARGOL2;
		// 	else if(controller.buttonY)
		// 		aimedState = ElevatorLevel.HATCHL3;
		// 	else if(controller.buttonX)
		// 		aimedState = ElevatorLevel.CARGOSHIP;
		// }
		// else
		// {
			if(controller.buttonA)
				aimedState = ElevatorLevel.BOTTOM;     //if hatch
			else if(controller.buttonB)
				aimedState = ElevatorLevel.HATCHL2;
			else if(controller.buttonY)
				aimedState = ElevatorLevel.HATCHL3;
			else if(controller.buttonX)
				aimedState = ElevatorLevel.MINROTATE;
		//}
	}

	public static void runElevator()
	{
		setElevatorEncoder();
		updateLivePosition();   
		if(aimedState != null)
		{
			System.out.println("Elevator moving to: " + aimedState);
			switch(aimedState) //check if aimed state has a value
			{
				case MANUAL:
					if(!manualOverride)
					{
						overrideValue = 0;
					}
					moveManual(overrideValue);
					break;
				case BOTTOM:
					moveToBottom();
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
	public static void moveToBottom()
	{
		if(stateDetection(ElevatorLevel.BOTTOM))
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
    
    public static void moveToCargoHandoff()
    {
		setPosition(Constants.elevatorCargoHandoff);
    }
    
    public static void moveToHatchHandoff()
    {
        setPosition(Constants.elevatorHatchHandoff);
    }

    public static void moveToHatchL2()
    {
        setPosition(Constants.elevatorHatchL2);
    }

    public static void moveToHatchL3()
    {
        setPosition(Constants.elevatorHatchL3);
    }

    public static void moveToCargoL2()
    {
        setPosition(Constants.elevatorCargoL2);
    }

    public static void moveToCargoShip()
    {
        setPosition(Constants.elevatorCargoShip);
    }

    public static void moveToStowed()
    {
        setPosition(Constants.elevatorStowed);
	}
	
	public static void moveToVerticalStowed()
	{
		setPosition(Constants.elevatorVerticalStowed);
	}

	public static void moveToMinRotate()
	{
		setPosition(Constants.elevatorMinRotation);
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

	public static boolean positionThreshold(double constant)
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

	public static void updateLivePosition()
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

	public static boolean stateDetection(ElevatorLevel level)
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

	public static void setEncoderValue(int encoderValue)
	{
		elevatorMaster.setSelectedSensorPosition(encoderValue, 0, Constants.kTimeoutMs);
	}
	
	public static void resetElevatorEncoder()
	{
		elevatorMaster.setSelectedSensorPosition(0,Constants.interstagePID, Constants.kTimeoutMs);
	}
	//----------------------------------------------------------

	public static boolean getLimitSwitch()
	{
		return limitSwitch.get();
	}
	
	public static void stopElevator()
	{
		elevatorMaster.stopMotor();
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
}