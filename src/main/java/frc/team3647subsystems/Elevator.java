package frc.team3647subsystems;

import frc.robot.*;
import frc.team3647inputs.Joysticks;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;


public class Elevator extends Subsystem
{
	public void initDefaultCommand()
	{
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
	}

	public enum ElevatorLevel
	{
		BOTTOM,
		LOW,
		MIDDLE,
		MAX
	}
	public int aimedElevatorState, elevatorEncoderValue, elevatorVelocity;
	public ElevatorLevel currentLevel;
	
	public DigitalInput bannerSensor = new DigitalInput(Constants.elevatorBannerSensor); 

    public WPI_TalonSRX GearboxMaster = new WPI_TalonSRX(Constants.ElevatorGearboxSRX); //8
	public VictorSPX GearboxSPX1 = new VictorSPX(Constants.ElevatorGearboxSPX1);	//12
    public VictorSPX GearboxSPX2 = new VictorSPX(Constants.ElevatorGearboxSPX2); //13
    
    public boolean bottom, sWitch, scale, lowerScale, moving, manualOverride, originalPositionButton;
	public double overrideValue;
	
	public int encoderState;
	public int manualEncoderValue;
	public int manualAdjustment;
    
    public void elevatorInitialization()
	{
		setElevatorLevel(ElevatorLevel.BOTTOM);
        //Config Sensors for Motors
        GearboxMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs);
		GearboxMaster.setSensorPhase(true);

		// Configure PID Values
		GearboxMaster.selectProfileSlot(Constants.interstagePIDIdx, 0);

		GearboxMaster.config_IntegralZone(Constants.interstagePIDIdx, Constants.interstageIZone, Constants.kTimeoutMs);
		GearboxMaster.config_kP(Constants.interstagePIDIdx, Constants.interstageP, Constants.kTimeoutMs);		
		GearboxMaster.config_kI(Constants.interstagePIDIdx, Constants.interstageI, Constants.kTimeoutMs);	
		GearboxMaster.config_kD(Constants.interstagePIDIdx, Constants.interstageD, Constants.kTimeoutMs);
		GearboxMaster.config_kF(Constants.interstagePIDIdx, Constants.interstageF, Constants.kTimeoutMs);
		
		// GearboxMaster.config_IntegralZone(Constants.carriagePIDX, Constants.carriageIZone, Constants.kTimeoutMs);
		// GearboxMaster.config_kP(Constants.carriagePIDX, Constants.carriageP, Constants.kTimeoutMs);
		// GearboxMaster.config_kI(Constants.carriagePIDX, Constants.carriageI, Constants.kTimeoutMs);
		// GearboxMaster.config_kD(Constants.carriagePIDX, Constants.carriageD, Constants.kTimeoutMs);	
		// GearboxMaster.config_kF(Constants.carriagePIDX, Constants.carriageF, Constants.kTimeoutMs);
		
		// //Motion Magic Constants
		// GearboxMaster.configMotionCruiseVelocity(Constants.elevatorCruiseVelocity, Constants.kTimeoutMs);
        // GearboxMaster.configMotionAcceleration(Constants.elevatorAcceleration, Constants.kTimeoutMs);

        GearboxSPX2.follow(GearboxMaster);
        GearboxSPX1.follow(GearboxMaster);
		GearboxSPX2.setInverted(false);
		GearboxMaster.setInverted(false);
		GearboxSPX1.setInverted(false);
	}

	public void configurePIDFMM(double p, double i, double d, double f, int vel, int accel)
	{
		GearboxMaster.selectProfileSlot(Constants.interstagePIDIdx, 0);

		GearboxMaster.config_IntegralZone(Constants.interstagePIDIdx, Constants.interstageIZone, Constants.kTimeoutMs);
		GearboxMaster.config_kP(Constants.interstagePIDIdx, p, Constants.kTimeoutMs);		
		GearboxMaster.config_kI(Constants.interstagePIDIdx, i, Constants.kTimeoutMs);	
		GearboxMaster.config_kD(Constants.interstagePIDIdx, d, Constants.kTimeoutMs);
		GearboxMaster.config_kF(Constants.interstagePIDIdx, f, Constants.kTimeoutMs);
		
		// GearboxMaster.config_IntegralZone(Constants.carriagePIDIdx, Constants.carriageIZone, Constants.kTimeoutMs);
		// GearboxMaster.config_kP(Constants.carriagePIDIdx, Constants.carriageP, Constants.kTimeoutMs);
		// GearboxMaster.config_kI(Constants.carriagePIDIdx, Constants.carriageI, Constants.kTimeoutMs);
		// GearboxMaster.config_kD(Constants.carriagePIDIdx, Constants.carriageD, Constants.kTimeoutMs);	
		// GearboxMaster.config_kF(Constants.carriagePIDIdx, Constants.carriageF, Constants.kTimeoutMs);
		
		//Motion Magic Constants
		GearboxMaster.configMotionCruiseVelocity(vel, Constants.kTimeoutMs);
        GearboxMaster.configMotionAcceleration(accel, Constants.kTimeoutMs);
        
	}

	/**
	 * 
	 * @param inputLevel BOTTOM, LOW, MIDDLE, MAX
	 */
	public void setElevatorLevel(ElevatorLevel inputLevel)
	{
		if(inputLevel == ElevatorLevel.BOTTOM)
		{
			resetElevator();
			currentLevel = ElevatorLevel.BOTTOM;
		}

		else if(inputLevel == ElevatorLevel.LOW)
		{
			setElevatorPosition(Constants.elevatorLow);
			currentLevel = ElevatorLevel.LOW;
		}

		else if(inputLevel == ElevatorLevel.MIDDLE)
		{
			setElevatorPosition(Constants.elevatorMiddle);
			currentLevel = ElevatorLevel.MIDDLE;
		}

		else if(inputLevel == ElevatorLevel.MAX)
		{
			setElevatorPosition(Constants.elevatorHigh);
			currentLevel = ElevatorLevel.MAX;
		}

		else
			System.out.println("INVALID ARM POSITION");
	}

	public void resetElevator()
	{
		if(reachedBottom())
		{
			stopElevator();
			resetElevatorEncoder();
		}
		else
		{
			moveElevator(-.25);
		}
	}

    private void setElevatorPosition(double positionInput)
    {
		//Motion Magic
        GearboxMaster.set(ControlMode.MotionMagic, positionInput);
    }

    public void stopElevator()
    {
        //Stop Elevator
		GearboxMaster.stopMotor();
    }

    public void moveElevator(double speed)
    {
		// Percent Output		
		GearboxMaster.set(ControlMode.PercentOutput, speed);
		if(bannerSensor.get())
			resetElevatorEncoder();
    }

    public void moveManual(Joysticks controller)
	{
        if(Math.abs(controller.leftJoyStickY) > 0)
        {
            moveElevator(controller.leftJoyStickY);
        }
        else
        {
            stopElevator();
        }
    }
	
	public void resetElevatorEncoder()
	{
        GearboxMaster.setSelectedSensorPosition(0,0,0);
	}

	public boolean reachedBottom()//false/true for comp, true/false for prac
	{
		if(bannerSensor.get())
		{
			resetElevatorEncoder();
			return true;
		}
        else
            return false;

	}

	public void testElevatorEncoders()
    {
        System.out.println("Elevator Encoder Value: " + elevatorEncoderValue + "Elevator Velocity: " + elevatorVelocity);
	}

	public void testBannerSensor()
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

    public void testElevatorCurrent()
    {
        System.out.println("Right Elevator Current:" + GearboxMaster.getOutputCurrent());
	}
	

}