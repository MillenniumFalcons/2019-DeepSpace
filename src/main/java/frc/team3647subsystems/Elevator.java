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
	public ElevatorLevel currentState, aimedState;
	
	private DigitalInput bannerSensor = new DigitalInput(Constants.elevatorBannerSensor); 

    private WPI_TalonSRX GearboxMaster = new WPI_TalonSRX(Constants.ElevatorGearboxSRX); //8
	private VictorSPX GearboxSPX1 = new VictorSPX(Constants.ElevatorGearboxSPX1);	//12
    private VictorSPX GearboxSPX2 = new VictorSPX(Constants.ElevatorGearboxSPX2); //13
    
    private boolean bottom, sWitch, scale, lowerScale, moving, manualOverride, originalPositionButton;
	private double overrideValue;
	
	private int encoderState;
	private int manualEncoderValue;
	private int manualAdjustment;

	private int encoderValue = getSelectedSensorPosition();
	private int prevEncoderValue;

    public Elevator()
	{
		aimedState = ElevatorLevel.BOTTOM;
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

	private boolean stateThreshold(double comparison, double input, double threshold)
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

	private boolean stateRecognizer(ElevatorLevel level)
	{
		int threshold = Constants.elevatorEncoderThreshold;
		switch(level)
		{
			case LOW:
				if(stateThreshold(Constants.elevatorLow, this.encoderValue, threshold))
					return true;
				else
					return false;
			case MIDDLE:
				if(stateThreshold(Constants.elevatorMiddle, this.encoderValue, threshold))
					return true;
				else
					return false;
			case MAX:
				if(stateThreshold(Constants.elevatorHigh, this.encoderValue, threshold))
					return true;
				else
					return false;
			case BOTTOM:
				return Robot.elevator.bannerSensor.get();
			default:
				return false;
		}
		// if(level == ElevatorLevel.LOW)
		// {
		// 	if(stateThreshold(Constants.elevatorLow, Robot.encoders.getElevatorEncoder(), threshold))
		// 		return true;
		// 	else
		// 		return false;
		// }
		// else if(level == ElevatorLevel.MIDDLE)
		// {
		// 	if(stateThreshold(Constants.elevatorMiddle, Robot.encoders.getElevatorEncoder(), threshold))
		// 		return true;
		// 	else
		// 		return false;
		// }
		// else if(level == ElevatorLevel.MAX)
		// {
		// 	if(stateThreshold(Constants.elevatorHigh, Robot.encoders.getElevatorEncoder(), threshold))
		// 		return true;
		// 	else
		// 		return false;
		// }
		// else if(level == ElevatorLevel.BOTTOM)
		// 	return Robot.elevator.bannerSensor.get();
		// else
		// 	return false;
	}

	public void runElevator(Joysticks controller)
	{
		// if(elevatorEncoderValue > Constants.elevatorSafetyLimit && overrideValue != 0)
		// {
		// 	currentWristState = 0;
		// 	aimedElevatorState = -10;
		// }
		// else if(elevatorEncoderValue > Constants.elevatorSafetyLimit && overrideValue == 0)
		// {
		// 	currentWristState = 0;
		// 	aimedElevatorState = -11;
		// }
		// else if(manualOverride)
		// {
		// 	currentWristState = 0;
		// 	aimedElevatorState = -1;
		// }
		/*else*/ 
		if(controller.buttonA)
		{
			aimedState = ElevatorLevel.BOTTOM;
		}
		else if(controller.buttonX)
		{
			aimedState = ElevatorLevel.LOW;
		}
		else if(controller.buttonB)
		{
			aimedState = ElevatorLevel.MIDDLE;
		}
		else if(controller.buttonY)
		{
			aimedState = ElevatorLevel.MAX;
		}
		switch(aimedState)
		{
			case BOTTOM:
				setElevatorPosition(ElevatorLevel.BOTTOM);
				if(stateRecognizer(aimedState))
					currentState = aimedState;
				break;
			case LOW:
				setElevatorPosition(ElevatorLevel.LOW);
				if(stateRecognizer(aimedState))
					currentState = aimedState;
				break;
			case MIDDLE:
				setElevatorPosition(ElevatorLevel.MIDDLE);
				if(stateRecognizer(aimedState))
					currentState = aimedState;
				break;
			case MAX:
				setElevatorPosition(ElevatorLevel.MAX);
				if(stateRecognizer(aimedState))
					currentState = aimedState;
				break;
			default:
				setElevatorPosition(ElevatorLevel.BOTTOM);
				if(stateRecognizer(aimedState))
					currentState = aimedState;
				break;
		}
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

	// /**
	//  * 
	//  * @param inputLevel BOTTOM, LOW, MIDDLE, MAX
	//  */
	// public void setElevatorPosition(ElevatorLevel inputLevel)
	// {
	// 	if(inputLevel == ElevatorLevel.BOTTOM)
	// 	{
	// 		resetElevator();
	// 		currentState = ElevatorLevel.BOTTOM;
	// 	}

	// 	else if(inputLevel == ElevatorLevel.LOW)
	// 	{
	// 		setElevatorPosition(Constants.elevatorLow);
	// 		currentState = ElevatorLevel.LOW;
	// 	}

	// 	else if(inputLevel == ElevatorLevel.MIDDLE)
	// 	{
	// 		setElevatorPosition(Constants.elevatorMiddle);
	// 		currentState = ElevatorLevel.MIDDLE;
	// 	}

	// 	else if(inputLevel == ElevatorLevel.MAX)
	// 	{
	// 		setElevatorPosition(Constants.elevatorHigh);
	// 		currentState = ElevatorLevel.MAX;
	// 	}

	// 	else
	// 		System.out.println("INVALID ARM POSITION");
	// }

	public void resetElevator()
	{
		if(!reachedBottom())
		{
			moveElevator(-.25);
		}
		else
		{
			stopElevator();
			resetEncoder();
		}
		
	}

    public void setElevatorPosition(ElevatorLevel positionInput)
    {
		switch(positionInput)
		{
			case BOTTOM:
				resetElevator();
				break;
			case LOW:
				GearboxMaster.set(ControlMode.MotionMagic, Constants.elevatorLow);
				break;
			case MIDDLE:
				GearboxMaster.set(ControlMode.MotionMagic, Constants.elevatorMiddle);
				break;
			case MAX:
				GearboxMaster.set(ControlMode.MotionMagic, Constants.elevatorHigh);
			default:
				System.out.println("INVALID HATCH INTAKE POSITION INPUT");
				break;
		}
		// //Motion Magic
        // GearboxMaster.set(ControlMode.MotionMagic, positionInput);
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
			resetEncoder();
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
	

	public boolean reachedBottom()//false/true for comp, true/false for prac
	{
		if(bannerSensor.get())
			return true;
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
	
	private void updateEncoder()
	{
		this.prevEncoderValue = this.encoderValue;
		this.encoderValue = this.getSelectedSensorPosition();
	}

	public int getSelectedSensorPosition()
	{
		return this.GearboxMaster.getSelectedSensorPosition(Constants.hatchIntakePIDIdx);
	}

	public void setEncoderPosition()
	{
		GearboxMaster.setSelectedSensorPosition(0);
	}

	public void resetEncoder()
	{
		GearboxMaster.setSelectedSensorPosition(0, Constants.hatchIntakePIDIdx, Constants.kTimeoutMs);
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

	public boolean getBannerSensor()
	{
		return bannerSensor.get();
	}
}