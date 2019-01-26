package frc.team3647subsystems;

import frc.robot.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;


public class Elevator
{
	public static int aimedElevatorState, elevatorEncoderValue, elevatorVelocity;
	public static int currentLevel;
	
	public static DigitalInput bannerSensor = new DigitalInput(Constants.elevatorBannerSensor); 

    public static WPI_TalonSRX GearboxMaster = new WPI_TalonSRX(Constants.leftGearboxSRX);
	public static VictorSPX leftGearboxSPX = new VictorSPX(Constants.leftGearboxSPX);
    public static VictorSPX rightGearboxSPX = new VictorSPX(Constants.rightGearboxSPX);
    
    public static boolean bottom, sWitch, scale, lowerScale, moving, manualOverride, originalPositionButton;
    public static double overrideValue;
    
    // 8 levels
    public static void elevatorInitialization()
	{
		setElevatorLevel(1);
        //Config Sensors for Motors
        GearboxMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs);
		GearboxMaster.setSensorPhase(true); //if i set to false I might not need to invert gearbox motors

		//Configure PID Values
		/*leftGearboxMaster.selectProfileSlot(Constants.interstagePID, 0);
		leftGearboxMaster.config_kF(Constants.carriagePID, Constants.carriageF, Constants.kTimeoutMs);
		leftGearboxMaster.config_kP(Constants.carriagePID, Constants.carriageP, Constants.kTimeoutMs);
		leftGearboxMaster.config_kI(Constants.carriagePID, Constants.carriageI, Constants.kTimeoutMs);
		leftGearboxMaster.config_kD(Constants.carriagePID, Constants.carriageD, Constants.kTimeoutMs);	
		leftGearboxMaster.config_IntegralZone(Constants.carriagePID, Constants.carriageIZone, Constants.kTimeoutMs);

		leftGearboxMaster.config_kF(Constants.interstagePID, Constants.interstageF, Constants.kTimeoutMs);		
		leftGearboxMaster.config_kP(Constants.interstagePID, Constants.interstageP, Constants.kTimeoutMs);		
		leftGearboxMaster.config_kI(Constants.interstagePID, Constants.interstageI, Constants.kTimeoutMs);		
       	leftGearboxMaster.config_kD(Constants.interstagePID, Constants.interstageD, Constants.kTimeoutMs);	
		leftGearboxMaster.config_IntegralZone(Constants.interstagePID, Constants.interstageIZone, Constants.kTimeoutMs);
		
		//Motion Magic Constants
		leftGearboxMaster.configMotionCruiseVelocity(Constants.elevatorCruiseVelocity, Constants.kTimeoutMs);
        leftGearboxMaster.configMotionAcceleration(Constants.elevatorAcceleration, Constants.kTimeoutMs);
        */

        rightGearboxSPX.follow(GearboxMaster);
        leftGearboxSPX.follow(GearboxMaster);
		rightGearboxSPX.setInverted(true);
		GearboxMaster.setInverted(true);
		leftGearboxSPX.setInverted(true);
	}

	/**
	 * 
	 * @param position 1 = low; 2 = middle; 3 = high goal
	 */
	public static void setElevatorLevel(int position)
	{
		if(position == 1)
		{
			setElevatorPosition(Constants.elevatorLow);
			currentLevel = 1;
		}
		else if(position == 2)
		{
			setElevatorPosition(Constants.elevatorMiddle);
			currentLevel = 2;
		}
		else if(position == 3)
		{
			setElevatorPosition(Constants.elevatorHigh);
			currentLevel = 3;
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

    public static int encoderState;
	public static int manualEncoderValue;
	public static int manualAdjustment;

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