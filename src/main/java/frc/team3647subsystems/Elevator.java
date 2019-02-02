package frc.team3647subsystems;

import frc.robot.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;


public class Elevator
{
	public static enum ElevatorLevel
	{
		LOW,
		MIDDLE,
		HIGH
	}
	public static int aimedElevatorState, elevatorEncoderValue, elevatorVelocity;
	public static ElevatorLevel currentLevel;
	
	public static DigitalInput bannerSensor = new DigitalInput(Constants.elevatorBannerSensor); 

    public static WPI_TalonSRX GearboxMaster = new WPI_TalonSRX(Constants.leftGearboxSRX);
	public static VictorSPX leftGearboxSPX = new VictorSPX(Constants.leftGearboxSPX);
    public static VictorSPX rightGearboxSPX = new VictorSPX(Constants.rightGearboxSPX);
    
    public static boolean bottom, sWitch, scale, lowerScale, moving, manualOverride, originalPositionButton;
	public static double overrideValue;
	
	public static int encoderState;
	public static int manualEncoderValue;
	public static int manualAdjustment;
    
    // 8 levels
    public static void elevatorInitialization()
	{
		setElevatorLevel(ElevatorLevel.LOW);
        //Config Sensors for Motors
        GearboxMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs);
		GearboxMaster.setSensorPhase(true); //if i set to false I might not need to invert gearbox motors

		//Configure PID Values
		GearboxMaster.selectProfileSlot(Constants.interstagePIDX, 0);

		GearboxMaster.config_IntegralZone(Constants.interstagePIDX, Constants.interstageIZone, Constants.kTimeoutMs);
		GearboxMaster.config_kP(Constants.interstagePIDX, Constants.interstageP, Constants.kTimeoutMs);		
		GearboxMaster.config_kI(Constants.interstagePIDX, Constants.interstageI, Constants.kTimeoutMs);	
		GearboxMaster.config_kD(Constants.interstagePIDX, Constants.interstageD, Constants.kTimeoutMs);
		GearboxMaster.config_kF(Constants.interstagePIDX, Constants.interstageF, Constants.kTimeoutMs);
		
		GearboxMaster.config_IntegralZone(Constants.carriagePIDX, Constants.carriageIZone, Constants.kTimeoutMs);
		GearboxMaster.config_kP(Constants.carriagePIDX, Constants.carriageP, Constants.kTimeoutMs);
		GearboxMaster.config_kI(Constants.carriagePIDX, Constants.carriageI, Constants.kTimeoutMs);
		GearboxMaster.config_kD(Constants.carriagePIDX, Constants.carriageD, Constants.kTimeoutMs);	
		GearboxMaster.config_kF(Constants.carriagePIDX, Constants.carriageF, Constants.kTimeoutMs);
		
		//Motion Magic Constants
		GearboxMaster.configMotionCruiseVelocity(Constants.elevatorCruiseVelocity, Constants.kTimeoutMs);
        GearboxMaster.configMotionAcceleration(Constants.elevatorAcceleration, Constants.kTimeoutMs);

        rightGearboxSPX.follow(GearboxMaster);
        leftGearboxSPX.follow(GearboxMaster);
		rightGearboxSPX.setInverted(true);
		GearboxMaster.setInverted(true);
		leftGearboxSPX.setInverted(true);
	}

	public void configurePIDFMM(double p, double i, double d, double f, double vel, double accel)
	{
		GearboxMaster.selectProfileSlot(Constants.interstagePIDX, 0);

		GearboxMaster.config_IntegralZone(Constants.interstagePIDX, Constants.interstageIZone, Constants.kTimeoutMs);
		GearboxMaster.config_kP(Constants.interstagePIDX, Constants.interstageP, Constants.kTimeoutMs);		
		GearboxMaster.config_kI(Constants.interstagePIDX, Constants.interstageI, Constants.kTimeoutMs);	
		GearboxMaster.config_kD(Constants.interstagePIDX, Constants.interstageD, Constants.kTimeoutMs);
		GearboxMaster.config_kF(Constants.interstagePIDX, Constants.interstageF, Constants.kTimeoutMs);
		
		GearboxMaster.config_IntegralZone(Constants.carriagePIDX, Constants.carriageIZone, Constants.kTimeoutMs);
		GearboxMaster.config_kP(Constants.carriagePIDX, Constants.carriageP, Constants.kTimeoutMs);
		GearboxMaster.config_kI(Constants.carriagePIDX, Constants.carriageI, Constants.kTimeoutMs);
		GearboxMaster.config_kD(Constants.carriagePIDX, Constants.carriageD, Constants.kTimeoutMs);	
		GearboxMaster.config_kF(Constants.carriagePIDX, Constants.carriageF, Constants.kTimeoutMs);
		
		//Motion Magic Constants
		GearboxMaster.configMotionCruiseVelocity(Constants.elevatorCruiseVelocity, Constants.kTimeoutMs);
        GearboxMaster.configMotionAcceleration(Constants.elevatorAcceleration, Constants.kTimeoutMs);
        
	}

	/**
	 * 
	 * @param position 1 = low; 2 = middle; 3 = high goal
	 */
	public static void setElevatorLevel(ElevatorLevel inputLevel)
	{
		if(inputLevel == ElevatorLevel.LOW)
		{
			setElevatorPosition(Constants.elevatorLow);
			currentLevel = ElevatorLevel.LOW;
		}
		else if(inputLevel == ElevatorLevel.MIDDLE)
		{
			setElevatorPosition(Constants.elevatorMiddle);
			currentLevel = ElevatorLevel.MIDDLE;
		}
		else if(inputLevel == ElevatorLevel.HIGH)
		{
			setElevatorPosition(Constants.elevatorHigh);
			currentLevel = ElevatorLevel.HIGH;
		}
		else
			System.out.println("INVALID ARM POSITION");
	}

    private static void setElevatorPosition(double positionInput)
    {
		//Motion Magic
        GearboxMaster.set(ControlMode.MotionMagic, positionInput);
    }

    public static void stopElevator()
    {
        //Stop Elevator
		GearboxMaster.stopMotor();
    }

    public static void moveElevator(double speed)
    {
		// Percent Output
        GearboxMaster.set(ControlMode.PercentOutput, speed);
    }

    public static void moveManual(double jValue)
	{
		if(jValue > 0)
		{
			moveElevator(overrideValue * 0.65);
			manualAdjustment = 1500;
			encoderState = 0;
		}
		else if(jValue < 0)
		{
			moveElevator(overrideValue * 0.2);
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
					setElevatorPosition(manualEncoderValue);
					break;
			}
		}
    }
    
	public static void setElevatorEncoder()
	{
        if(reachedBottom())
		{
            resetElevatorEncoders();
		}
		elevatorEncoderValue = GearboxMaster.getSelectedSensorPosition(0);
		elevatorVelocity = GearboxMaster.getSelectedSensorVelocity(0);
	}
	
	public static void resetElevatorEncoders()
	{
        GearboxMaster.getSensorCollection().setQuadraturePosition(0, 10);
	}

	public static boolean reachedBottom()//false/true for comp, true/false for prac
	{
        if(bannerSensor.get())
            return false;// true for prac, false for comp 
        else
            return true;

	}

	public static void setElevatorButtons(boolean Button)
	{
		//supposedly 8 levels
	}

	public static void setManualOverride(double jValue)
	{
        if(Math.abs(jValue) <.2 )
		{
			manualOverride = false;
		}
		else
		{
            overrideValue = jValue;
			manualOverride = true;
		}
	}

	public static void testElevatorEncoders()
    {
        System.out.println("Elevator Encoder Value: " + elevatorEncoderValue + "Elevator Velocity: " + elevatorVelocity);
	}

	public static void testBannerSensor()
    {
        if(reachedBottom())
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
        System.out.println("Right Elevator Current:" + GearboxMaster.getOutputCurrent());
	}
	

}