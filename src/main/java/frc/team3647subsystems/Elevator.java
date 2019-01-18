package frc.team3647subsystems;

import frc.robot.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Elevator
{
    public static int aimedElevatorState, elevatorEncoderValue, elevatorVelocity;

    public static WPI_TalonSRX leftGearboxMaster = new WPI_TalonSRX(Constants.leftGearboxSRX);
	public static WPI_TalonSRX rightGearboxSRX = new WPI_TalonSRX(Constants.rightGearboxSRX);
	public static VictorSPX leftGearboxSPX = new VictorSPX(Constants.leftGearboxSPX);
    public static VictorSPX rightGearboxSPX = new VictorSPX(Constants.rightGearboxSPX);
    
    public static boolean bottom, sWitch, scale, lowerScale, moving, manualOverride, originalPositionButton;
    public static double overrideValue;
    
    // 8 levels
    public static void elevatorInitialization()
	{
        //Config PID for Motors
        leftGearboxMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, Constants.kTimeoutMs);
		leftGearboxMaster.setSensorPhase(true); //if i set to false I might not need to invert gearbox motors

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
        rightGearboxSRX.follow(leftGearboxMaster);
        rightGearboxSPX.follow(leftGearboxMaster);
        leftGearboxSPX.follow(leftGearboxMaster);
        rightGearboxSRX.setInverted(true);
		rightGearboxSPX.setInverted(true);
		leftGearboxMaster.setInverted(true);
		leftGearboxSPX.setInverted(true);
    }

    public static void moveElevatorPosition(double position)
    {
        leftGearboxMaster.set(ControlMode.MotionMagic, position);
        //Motion Magic
    }

    public static void stopElevator()
    {
        //Stop Elevator
        leftGearboxMaster.stopMotor();
    }

    public static void moveElevator(double speed)
    {
		// Percent Output
		//
        leftGearboxMaster.set(ControlMode.PercentOutput, speed);
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
					moveElevatorPosition(manualEncoderValue);
					break;
			}
		}
    }
    


}