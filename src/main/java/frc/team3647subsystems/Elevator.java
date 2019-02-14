//Class not created by Kunal Singla

package frc.team3647subsystems;

import frc.robot.*;
import frc.team3647inputs.Joysticks;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;


public class Elevator
{

	public static ElevatorLevel currentState;
	public static ElevatorLevel aimedState;
	
	public static DigitalInput limitSwitch = new DigitalInput(Constants.elevatorBeamBreakPin); 

    public static WPI_TalonSRX elevatorMaster = new WPI_TalonSRX(Constants.ElevatorGearboxSRX); //8
	public static VictorSPX GearboxSPX1 = new VictorSPX(Constants.ElevatorGearboxSPX1);	//12
    public static VictorSPX GearboxSPX2 = new VictorSPX(Constants.ElevatorGearboxSPX2); //13

	public static int elevatorEncoderCCL, elevatorEncoderValue, elevatorEncoderVelocity;
	public static double overrideValue;
	public static boolean manualOverride;
    
    public static void elevatorInitialization()
	{
		aimedState = ElevatorLevel.START;
        //Config Sensors for Motors
        elevatorMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs);
		elevatorMaster.setSensorPhase(true);

		// Configure PID Values
		elevatorMaster.selectProfileSlot(Constants.interstagePID, 0);

		elevatorMaster.config_kP(Constants.interstagePID, Constants.interstageP, Constants.kTimeoutMs);		
		elevatorMaster.config_kI(Constants.interstagePID, Constants.interstageI, Constants.kTimeoutMs);	
		elevatorMaster.config_kD(Constants.interstagePID, Constants.interstageD, Constants.kTimeoutMs);
		elevatorMaster.config_kF(Constants.interstagePID, Constants.interstageF, Constants.kTimeoutMs);
		
		// GearboxMaster.config_IntegralZone(Constants.carriagePIDX, Constants.carriageIZone, Constants.kTimeoutMs);
		// GearboxMaster.config_kP(Constants.carriagePIDX, Constants.carriageP, Constants.kTimeoutMs);
		// GearboxMaster.config_kI(Constants.carriagePIDX, Constants.carriageI, Constants.kTimeoutMs);
		// GearboxMaster.config_kD(Constants.carriagePIDX, Constants.carriageD, Constants.kTimeoutMs);	
		// GearboxMaster.config_kF(Constants.carriagePIDX, Constants.carriageF, Constants.kTimeoutMs);
		
		// //Motion Magic Constants
		// GearboxMaster.configMotionCruiseVelocity(Constants.elevatorCruiseVelocity, Constants.kTimeoutMs);
        // GearboxMaster.configMotionAcceleration(Constants.elevatorAcceleration, Constants.kTimeoutMs);

        GearboxSPX2.follow(elevatorMaster);
        GearboxSPX1.follow(elevatorMaster);
		GearboxSPX2.setInverted(false);
		elevatorMaster.setInverted(false);
		GearboxSPX1.setInverted(false);
	}

	public static void configurePIDFMM(double p, double i, double d, double f, int vel, int accel)
	{
		elevatorMaster.selectProfileSlot(Constants.interstagePID, 0);

		elevatorMaster.config_kP(Constants.interstagePID, p, Constants.kTimeoutMs);		
		elevatorMaster.config_kI(Constants.interstagePID, i, Constants.kTimeoutMs);	
		elevatorMaster.config_kD(Constants.interstagePID, d, Constants.kTimeoutMs);
		elevatorMaster.config_kF(Constants.interstagePID, f, Constants.kTimeoutMs);
		
		// GearboxMaster.config_IntegralZone(Constants.carriagePIDIdx, Constants.carriageIZone, Constants.kTimeoutMs);
		// GearboxMaster.config_kP(Constants.carriagePIDIdx, Constants.carriageP, Constants.kTimeoutMs);
		// GearboxMaster.config_kI(Constants.carriagePIDIdx, Constants.carriageI, Constants.kTimeoutMs);
		// GearboxMaster.config_kD(Constants.carriagePIDIdx, Constants.carriageD, Constants.kTimeoutMs);	
		// GearboxMaster.config_kF(Constants.carriagePIDIdx, Constants.carriageF, Constants.kTimeoutMs);
		
		//Motion Magic Constants
		elevatorMaster.configMotionCruiseVelocity(vel, Constants.kTimeoutMs);
        elevatorMaster.configMotionAcceleration(accel, Constants.kTimeoutMs);
	}

	public enum ElevatorLevel
	{
		START,
        MANUAL,
		BOTTOM,
        CARGOHANDOFF,
        HATCHHANDOFF,
		HATCHL2,
		HATCHL3, //also cargo lvl3
        CARGOL2,
        CARGOSHIP,
		STOWED,
		VERTICALSTOWED, 
		MINARMROTATION
	}

	public static void runElevator(Joysticks controller)
	{
		setElevatorEncoder();
		setManualOverride(controller.rightJoyStickY);
		System.out.println("runElevator");
		if(controller.buttonA)
			aimedState = ElevatorLevel.BOTTOM;

		else if(controller.buttonB)
			aimedState = ElevatorLevel.CARGOHANDOFF;

		else if(controller.leftBumper)
			aimedState = ElevatorLevel.STOWED;

		else if(controller.rightBumper)
			aimedState = ElevatorLevel.VERTICALSTOWED;	

		else if(controller.buttonY)
			aimedState = ElevatorLevel.HATCHL2;
		
		else if(controller.dPadLeft)
			aimedState = ElevatorLevel.MINARMROTATION;

		else if(controller.buttonX)
			aimedState = ElevatorLevel.CARGOSHIP;
		
		else if(controller.dPadRight)
			aimedState = ElevatorLevel.CARGOL2;

		else if(controller.dPadUp)
			aimedState = ElevatorLevel.HATCHL3;

            
		switch(aimedState)
		{
			case MANUAL:
				if(!manualOverride)
				{
					overrideValue = 0;
				}
                moveManual(overrideValue);
				break;
			case START:
				moveToBottomStart();
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
			case MINARMROTATION:
				moveToMinArmRotation();
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

	public static void moveToBottom()
	{
		if(stateDetection(ElevatorLevel.BOTTOM))
		{
			stopElevator();
			resetElevatorEncoder();
		}
		else
		{
			setPosition(0);
		}	
	}
    
    public static void moveToCargoHandoff()
    {
        setPosition(Constants.elevatorrCargoHandoff);
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
	
	private static void moveToVerticalStowed()
	{
		setPosition(Constants.elevatorVerticalStowed);
	}

	private static void moveToMinArmRotation()
	{
		setPosition(Constants.elevatorMinRotation);
	}

	private static void moveToBottomStart()
	{
		if(stateDetection(ElevatorLevel.BOTTOM))
		{
			stopElevator();
			resetElevatorEncoder();
		}
		else
		{
			setOpenLoop(-.3);
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
	
    public static void setPosition(int position)
    {
        elevatorMaster.set(ControlMode.MotionMagic, position);
    }

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

	public static boolean stateDetection(ElevatorLevel level)
	{
        switch(level)
		{
            case BOTTOM:
                return getLimitSwitch();
            case CARGOHANDOFF:
                return positionThreshold(Constants.elevatorrCargoHandoff);
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
            default:
                return false;
        }
	}
	
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

	public static boolean getLimitSwitch()
	{
		return limitSwitch.get();
	}

	public static void setEncoderValue(int encoderValue)
	{
		elevatorMaster.setSelectedSensorPosition(encoderValue, 0, Constants.kTimeoutMs);
	}
	
	public static void resetElevatorEncoder()
	{
		elevatorMaster.setSelectedSensorPosition(0,Constants.interstagePID, Constants.kTimeoutMs);
	}
	
	public static void stopElevator()
	{
		elevatorMaster.stopMotor();
	}
	
	public static void testElevatorEncoders()
    {
        System.out.println("Elevator Encoder Value: " + elevatorEncoderValue + "Elevator Velocity: " + elevatorEncoderVelocity);
	}

	public static void testBannerSensor()
    {
        if(getLimitSwitch())
        {
            System.out.println("Banner Sensor Triggered!");
        }
        else
        {
            System.out.println("Banner Sensor Not Triggered!");
        }
    }

    public static void testElevatorCurrent()
    {
        System.out.println("Elevator Current:" + elevatorMaster.getOutputCurrent());
	}
	

}