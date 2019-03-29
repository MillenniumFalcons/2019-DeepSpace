package frc.team3647subsystems;

import frc.robot.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;


public class Elevator
{

	public static ElevatorLevel aimedState;
	private static boolean bannerSensor = false;
	
	// Sensor at bottom of elevator
	public static DigitalInput limitSwitch = new DigitalInput(Constants.elevatorBeamBreakPin);

	// Elevator motors
    public static WPI_TalonSRX elevatorMaster = new WPI_TalonSRX(Constants.ElevatorGearboxSRX); //8
	public static VictorSPX GearboxSPX1 = new VictorSPX(Constants.ElevatorGearboxSPX1);	//12
	public static VictorSPX GearboxSPX2 = new VictorSPX(Constants.ElevatorGearboxSPX2); //13
	public static VictorSPX GearBoxSPX3 = new VictorSPX(Constants.ElevatorGearboxSPX3);

	public static int encoderError, encoderValue, encoderVelocity;
	
	// private static double overrideValue;
	// private static boolean manualOverride;
    
    public static void init()
	{
		aimedState = ElevatorLevel.STOP;
		initSensors();
	}
	
	public static void initSensors()
	{
		//Config Sensors for encoder
		elevatorMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs);
		elevatorMaster.setSensorPhase(true);

		configurePIDFMM(Constants.interstagePIDF[0], Constants.interstagePIDF[1], Constants.interstagePIDF[2], 
			Constants.interstagePIDF[3], Constants.kElevatorCruiseVelocity, Constants.kElevatorAcceleration);

		GearboxSPX2.follow(elevatorMaster);
		GearboxSPX1.follow(elevatorMaster);
		GearBoxSPX3.follow(elevatorMaster);
		GearboxSPX2.setInverted(false);
		elevatorMaster.setInverted(false);
		GearboxSPX1.setInverted(false);

		Elevator.elevatorMaster.enableCurrentLimit(true);
		Elevator.elevatorMaster.configContinuousCurrentLimit(35);

		elevatorMaster.setNeutralMode(NeutralMode.Brake);
		elevatorMaster.setName("Elevator", "elevatorMaster");
		elevatorMaster.setExpiration(Constants.expirationTimeSRX);
		bannerSensor = limitSwitch.get();
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
		BOTTOM(-1),
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

	public static void updateBannerSensor()
	{
		bannerSensor = limitSwitch.get();
	}

	public static void run()
	{
		updateBannerSensor();
		if(aimedState != null)
		{
			if(aimedState.encoderVal != -1)
				setPosition(aimedState.encoderVal);
			else
			{
				switch(aimedState) //check if aimed state has a value
				{
					case BOTTOM:
						moveToBottom();
						break;
					case STOP:
						stop();
						break;
					case START:
						moveToBottomStart();
						break;
					default:
						break;
				}
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
		if(getBannerSenor())
		{
			stop();
			resetElevatorEncoder();
		}
		else
			setOpenLoop(-.3);
	}
	static boolean reachedZeroButNotBottom = false;
	private static void moveToBottom()
	{
		if(positionThreshold(0) && !getBannerSenor())
			reachedZeroButNotBottom = true;

		if(reachedZeroButNotBottom)
		{
			moveToBottomStart();
			reachedZeroButNotBottom = !getBannerSenor();
		}
		else if(!getBannerSenor() && !reachedZeroButNotBottom)
			setPosition(0);


	}

    
	//----------------------------------------------------------

	// //Manual movement-------------------------------------------

	// public static void setManualController(Joysticks controller)
	// {
	// 		if(controller.buttonA)
	// 			aimedState = ElevatorLevel.BOTTOM;     //if hatch
	// 		else if(controller.buttonB)
	// 			aimedState = ElevatorLevel.HATCHL2;
	// 		else if(controller.buttonY)
	// 			aimedState = ElevatorLevel.HATCHL3;
	// 		else if(controller.buttonX)
	// 			aimedState = ElevatorLevel.MINROTATE;
	// }

	// public static void setElevatorManualControl(Joysticks controller)
	// {
	// 	setManualOverride(controller.leftJoyStickY);
	// }


	// public static void setManualOverride(double jValue)
	// {
	// 	if(Math.abs(jValue) > .15) //deadzone
	// 	{
	// 		manualOverride = true;
	// 		aimedState = ElevatorLevel.MANUAL;
    //         overrideValue = jValue;
	// 	} 
	// 	else 
	// 	{
	// 		manualOverride = false;
	// 	}
	// }

	// private static int encoderState, manualAdjustment, manualEncoderValue;

	// public static void moveManual(double jValue)
	// {
	// 	updateEncoder();
	// 	// updateLivePosition();
	// 	if(jValue > 0)
	// 	{
	// 		setOpenLoop(jValue * 0.5);
	// 		manualAdjustment = 50;
	// 		encoderState = 0;
	// 	}
	// 	else if(jValue < 0)
	// 	{
	// 		setOpenLoop(jValue * 0.5);
	// 		manualAdjustment = 0;
	// 		encoderState = 0;
	// 	}
	// 	else
	// 	{
	// 		switch(encoderState)
	// 		{
	// 			case 0:
	// 				manualEncoderValue = encoderValue + manualAdjustment;
	// 				encoderState = 1;
	// 				break;
	// 			case 1:
	// 				setPosition(manualEncoderValue);
	// 				break;
	// 		}
	// 	}
	// }

    public static void setOpenLoop(double power)
    {
		elevatorMaster.set(ControlMode.PercentOutput, power);
	}
	//----------------------------------------------------------

	private static boolean positionThreshold(double constant)
	{
		return (constant + Constants.kElevatorPositionThreshold) > encoderValue && 
				(constant - Constants.kElevatorPositionThreshold) < encoderValue;
	}

	public static boolean reachedState(ElevatorLevel nAimedState)
	{
		if(aimedState != null && nAimedState.encoderVal != -1)
			return positionThreshold(nAimedState.encoderVal);
		return false;
	}

	public static boolean reachedAimedState()
	{
		return reachedState(aimedState);
	}

	//Encoder Methods-------------------------------------------
	public static void updateEncoder()
	{
		if(bannerSensor)
			resetElevatorEncoder();
		encoderValue = elevatorMaster.getSelectedSensorPosition(0);
		encoderVelocity = elevatorMaster.getSelectedSensorVelocity(0);
		encoderError = elevatorMaster.getClosedLoopError(0);
	}

	private static void setEncoderValue(int encoderValue)
	{
		elevatorMaster.setSelectedSensorPosition(encoderValue, 0, Constants.kTimeoutMs);
	}
	
	private static void resetElevatorEncoder()
	{
		setEncoderValue(0);
	}
	//----------------------------------------------------------

	private static boolean getBannerSenor()
	{
		return bannerSensor;
	}
	
	public static void stop()
	{
		elevatorMaster.stopMotor();
	}

	public static boolean isAboveMinRotate(int threshold)
	{
		return (encoderValue >= Constants.elevatorMinRotation + threshold);
	}

	private static boolean isValueAboveMinRotate(int val)
	{
		return (val >= Constants.elevatorMinRotation - 500);
	}
	public static boolean isStateAboveMinRotate(ElevatorLevel state)
	{
		if(state != null)
			return isValueAboveMinRotate(state.encoderVal);
		return false;
	}
	
	public static void printElevatorEncoders()
    {
        System.out.println("Elevator Encoder Value: " + encoderValue );
	}

	public static void printBannerSensor()
    {
    	System.out.println("Banner Sensor Value: " + getBannerSenor());
    }

    public static void printElevatorCurrent()
    {
        System.out.println("Elevator Current: " + elevatorMaster.getOutputCurrent());
	}

	public static void disableElevator()
	{
		aimedState = null;
		elevatorMaster.setNeutralMode(NeutralMode.Coast);
		stop();
	}
}