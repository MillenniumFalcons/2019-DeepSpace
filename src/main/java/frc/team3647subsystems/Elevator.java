
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
	
	public static DigitalInput beamSensor = new DigitalInput(Constants.elevatorBreamBreakPin); 

    public static WPI_TalonSRX elevatorMaster = new WPI_TalonSRX(Constants.ElevatorGearboxSRX); //8
	public static VictorSPX GearboxSPX1 = new VictorSPX(Constants.ElevatorGearboxSPX1);	//12
    public static VictorSPX GearboxSPX2 = new VictorSPX(Constants.ElevatorGearboxSPX2); //13

	public static int elevatorEncoderCCL, elevatorEncoderValue, elevatorEncoderVelocity;
	public static double overrideValue;
	public static boolean manualOverride;
    
    public void elevatorInitialization()
	{
		aimedState = ElevatorLevel.BOTTOM;
        //Config Sensors for Motors
        elevatorMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs);
		elevatorMaster.setSensorPhase(true);

		// Configure PID Values
		elevatorMaster.selectProfileSlot(Constants.interstagePID, 0);

		elevatorMaster.config_IntegralZone(Constants.interstagePID, Constants.interstageIdx, Constants.kTimeoutMs);
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

	public void configurePIDFMM(double p, double i, double d, double f, int vel, int accel)
	{
		elevatorMaster.selectProfileSlot(Constants.interstagePID, 0);

		elevatorMaster.config_IntegralZone(Constants.interstagePID, Constants.interstageIdx, Constants.kTimeoutMs);
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
		BOTTOM,
		HANDOFF,
		LOW,
		MIDDLE,
		MAX
	}

	// public static void runElevator(Joysticks controller)
	// {
	// 	setElevatorEncoder();
	// 	setManualOverride(controller.leftJoyStickY);
	// 	//get joy input
	// 	if(manualOverride)
	// 		aimedState = WristPosition.MANUAL;
	// 	else if(controller.dPadUp)
	// 		aimedState = WristPosition.SCORE;
	// 	else if(controller.dPadDown)
	// 		aimedState = WristPosition.STOWED;
	// 	else if(controller.dPadSide)
	// 		aimedState = WristPosition.GROUND;

	// 	//move to positions
	// 	switch(aimedState)
	// 	{
	// 		case MANUAL:
	// 			if(!manualOverride)
	// 			{
	// 				overrideValue = 0;
	// 			}
	// 			moveManual(overrideValue);
	// 		case STOWED:
	// 			closeClamp();
	// 			moveToStowed();
	// 			break;
	// 		case SCORE:
	// 			closeClamp();
	// 			moveToScore();
	// 			break;
	// 		case GROUND:
	// 			moveToGround();
	// 			openClamp();
	// 			break;
	// 		default:
	// 			break;
	// 	}
	// }
	
	public static void setManualOverride(double jValue)
	{
		if(Math.abs(jValue) < .05) //deadzone
		{
			manualOverride = false;
		} 
		else 
		{
			overrideValue = jValue;
			manualOverride = true;
		}
	}

	public void moveToBottom()
	{
		if(stateDetection(ElevatorLevel.BOTTOM))
		{
			stopElevator();
			resetElevatorEncoder();
		}
		else
		{
			setOpenLoop(-.35);
		}
		
    }
    
	private static boolean stateThreshold(double comparison, double input, double threshold)
	{
		if((comparison + threshold) > input && (comparison - threshold) > input)
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	public static boolean stateRecognizer(ElevatorLevel level)
	{
		int threshold = 25;
		if(level == ElevatorLevel.LOW)
		{
			if(stateThreshold(Constants.elevatorLow, elevatorEncoderValue, threshold))
				return true;
			else
				return false;
		}
		else if(level == ElevatorLevel.MIDDLE)
		{
			if(stateThreshold(Constants.elevatorMiddle, elevatorEncoderValue, threshold))
				return true;
			else
				return false;
		}
		else if(level == ElevatorLevel.MAX)
		{
			if(stateThreshold(Constants.elevatorHigh, elevatorEncoderValue, threshold))
				return true;
			else
				return false;
		}
		else if(level == ElevatorLevel.BOTTOM)
			return beamSensor.get();
		else
			return false;

	}

	public static int encoderState, manualAdjustment, manualEncoderValue;

	public static void moveManual(double jValue)
	{
		if(jValue > 0)
		{
			setOpenLoop(overrideValue * 0.65);
			manualAdjustment = 50;
			encoderState = 0;
		}
		else if(jValue < 0)
		{
			setOpenLoop(overrideValue * 0.2);
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

	private static boolean positionThreshold(double comparison, double input, double threshold)
	{
		if((comparison + threshold) > input && (comparison - threshold) > input)
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
		int threshold = 25;
		if(level == ElevatorLevel.LOW)
		{
			if(positionThreshold(Constants.elevatorLow, elevatorEncoderValue, threshold))
				return true;
			else
				return false;
		}
		else if(level == ElevatorLevel.MIDDLE)
		{
			if(positionThreshold(Constants.elevatorMiddle, elevatorEncoderValue, threshold))
				return true;
			else
				return false;
		}
		else if(level == ElevatorLevel.MAX)
		{
			if(positionThreshold(Constants.elevatorHigh, elevatorEncoderValue, threshold))
				return true;
			else
				return false;
		}
		else if(level == ElevatorLevel.BOTTOM)
			return getLimitSwitch();
		else
			return false;

	}

	public static void resetEncoder()
	{

	}
	
	public static void setElevatorEncoder()
	{
		if(getLimitSwitch())
		{
			resetEncoder();
		}
		elevatorEncoderValue = elevatorMaster.getSelectedSensorPosition(0);
		elevatorEncoderVelocity = elevatorMaster.getSelectedSensorVelocity(0);
		elevatorEncoderCCL = elevatorMaster.getClosedLoopError(0);
	}

	public static boolean getLimitSwitch()
	{
		if(beamSensor.get())
		{
			return false;
		}
		else
		{
			return true;
		}
	}

	public static void setEncoderValue(int encoderValue)
	{
		elevatorMaster.setSelectedSensorPosition(encoderValue, 0, Constants.kTimeoutMs);
	}
	
	public static void resetElevatorEncoder()
	{
		elevatorMaster.setSelectedSensorPosition(0,Constants.interstagePID, Constants.kTimeoutMs);
	}
	
	public void stopElevator()
	{
		elevatorMaster.stopMotor();
	}
	
	public static void testElevatorEncoders()
    {
        System.out.println("Elevator Encoder Value: " + elevatorEncoderValue + "Elevator Velocity: " + elevatorEncoderVelocity);
	}

	public static void testBannerSensor()
    {
        if(beamSensor.get())
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