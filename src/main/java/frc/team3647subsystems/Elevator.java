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
		HatchLevel1,
		HatchHandoff,
		BallHandoff,
		HatchIntakeMovement,
		RobotStowed,
		HatchLevel2,
		CargoLevel2,
		HatchLevel3,
	}
	//public int aimedElevatorState, elevatorEncoderValue, elevatorVelocity;
	private ElevatorLevel currentState, aimedState;
	
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
		aimedState = ElevatorLevel.RobotStowed;
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
			case HatchLevel1:
				return getBannerSensor();
			case HatchHandoff:
				if(stateThreshold(Constants.elevatorEncoderHatchHandoff, this.getEncoder(), threshold))
					return true;
				else
					return false;
			case BallHandoff:
				if(stateThreshold(Constants.elevatorEncoderBallHandoff, this.getEncoder(), threshold))
					return true;
				else
					return false;
			case HatchIntakeMovement:
				if(stateThreshold(Constants.elevatorEncoderHatchIntakeMovement, this.getEncoder(), threshold))
					return true;
				else
					return false;
			case RobotStowed:
				if(stateThreshold(Constants.elevatorEncoderRobotStowed, this.getEncoder(), threshold))
					return true;
				else
					return false;
			case HatchLevel2:
				if(stateThreshold(Constants.elevatorEncoderHatchLevel2, this.getEncoder(), threshold))
					return true;
				else
					return false;
			case CargoLevel2:
				if(stateThreshold(Constants.elevatorEncoderCargoLevel2, this.getEncoder(), threshold))
					return true;
				else
					return false;
			case HatchLevel3:
				if(stateThreshold(Constants.elevatorEncoderHatchLevel3, this.getEncoder(), threshold))
					return true;
				else
					return false;
			default:
				return false;
		}
	}

	public void runElevator()
	{
		switch(aimedState)
		{
			case HatchLevel1:
				setElevatorPosition(ElevatorLevel.HatchLevel1);
				if(stateRecognizer(aimedState))
					currentState = aimedState;
				break;
			case HatchHandoff:
				setElevatorPosition(ElevatorLevel.HatchHandoff);
					if(stateRecognizer(aimedState))
						currentState = aimedState;
				break;
			case BallHandoff:
				setElevatorPosition(ElevatorLevel.BallHandoff);
				if(stateRecognizer(aimedState))
					currentState = aimedState;
				break;
			case HatchIntakeMovement:
				setElevatorPosition(ElevatorLevel.HatchIntakeMovement);
						if(stateRecognizer(aimedState))
							currentState = aimedState;
				break;
			case RobotStowed:
				setElevatorPosition(ElevatorLevel.RobotStowed);
				if(stateRecognizer(aimedState))
					currentState = aimedState;
				break;
			case HatchLevel2:
				setElevatorPosition(ElevatorLevel.HatchLevel2);
				if(stateRecognizer(aimedState))
					currentState = aimedState;
				break;
			case CargoLevel2:
				setElevatorPosition(ElevatorLevel.CargoLevel2);
						if(stateRecognizer(aimedState))
							currentState = aimedState;
				break;
			case HatchLevel3:
				setElevatorPosition(ElevatorLevel.HatchLevel3);
						if(stateRecognizer(aimedState))
							currentState = aimedState;
				break;
			default:
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
			case HatchLevel1:
				this.resetElevator();
				break;
			case HatchHandoff:
				this.setMMPosition(Constants.elevatorEncoderHatchHandoff);
				break;
			case BallHandoff:
				this.setMMPosition(Constants.elevatorEncoderBallHandoff);
				break;
			case HatchIntakeMovement:
				this.setMMPosition(Constants.elevatorEncoderHatchIntakeMovement);
				break;
			case RobotStowed:
				this.setMMPosition(Constants.elevatorEncoderRobotStowed);
				break;
			case HatchLevel2:
				this.setMMPosition(Constants.elevatorEncoderHatchLevel2);
				break;
			case CargoLevel2:
				this.setMMPosition(Constants.elevatorEncoderCargoLevel2);
				break;
			case HatchLevel3:
				this.setMMPosition(Constants.elevatorEncoderHatchLevel3);
				break;
			default:
				break;
		}
    }

    public void stopElevator()
    {
        //Stop Elevator
		GearboxMaster.stopMotor();
	}
	
	private void setMMPosition(int encoderVal)
	{
		GearboxMaster.set(ControlMode.MotionMagic, encoderVal);
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
        System.out.println("Elevator Encoder Value: " + getEncoder());
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

	public void setAimedState(ElevatorLevel aimedState)
	{
		this.aimedState = aimedState;
	}

	public ElevatorLevel getAimedState()
	{
		return this.aimedState;
	}

	public ElevatorLevel getCurrentState()
	{
		return this.currentState;
	}
}