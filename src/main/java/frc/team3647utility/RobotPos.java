package frc.team3647utility;

import frc.team3647subsystems.*;
import frc.robot.*;

import edu.wpi.first.wpilibj.Notifier;

public class RobotPos
{
    public Elevator.ElevatorLevel eLevel;
    public Arm.ArmPosition armPos;

    public enum Movement
    {
        ARRIVED,
        MOVEELEV,
        MOVEARM,
        SAFEZMOVE
    }

    public RobotPos(Elevator.ElevatorLevel eLevel, Arm.ArmPosition armPos)
    {
        this.eLevel = eLevel;
        this.armPos = armPos;
    }

    public void initRobotPos()
    {
        Notifier robotPosThread = new Notifier(() ->
        {
            eLevel = Elevator.currentState;
            armPos = armPos;
        });
        robotPosThread.startPeriodic(0.01);
    }

    public Movement movementCheck(SeriesStateMachine.ScoringPosition scoringPos, RobotPos posData)
    {
        if(eLevel == posData.eLevel && armPos == posData.armPos)
        {
            return Movement.ARRIVED;
        }
        else if(eLevel != posData.eLevel && armPos == posData.armPos)
        {
            return Movement.MOVEELEV;
        }
        else if(eLevel == posData.eLevel && armPos != posData.armPos) //NEED TO SET A THREHSOLD HERE FOR WHEN ARM CAN MOVE WITHOUT SAFEZ
        {
            switch(scoringPos)
            {
                case HATCHL1FORWARDS:
                    if(threshold(Constants.armSRXFlatForwards, Arm.armEncoderValue, 1000))
                        return Movement.MOVEARM;
                    else
                        return Movement.SAFEZMOVE;
                case HATCHL1BACKWARDS:
                    if(threshold(Constants.armSRXFlatForwards, Arm.armEncoderValue, 1000))
                        return Movement.MOVEARM;
                    else
                        return Movement.SAFEZMOVE;
                case STOWED:
                    if(threshold(Constants.armSRXFlatForwards, Arm.armEncoderValue, 100))
                        return Movement.MOVEARM;
                    else
                        return Movement.SAFEZMOVE;    
                case HATCHHANDOFF:
                    if(Arm.armEncoderValue > Constants.armSRXHatchHandoff)
                        return Movement.MOVEARM;
                    else
                        return Movement.SAFEZMOVE;                
                case CARGOHANDOFF:
                    if(threshold(Constants.armSRXFlatForwards, Arm.armEncoderValue, 100))
                        return Movement.MOVEARM;
                    else
                        return Movement.SAFEZMOVE;
                case VERTICALSTOWED:
                    if(threshold(Constants.armSRXFlatForwards, Arm.armEncoderValue, 100))
                        return Movement.MOVEARM;
                    else
                        return Movement.SAFEZMOVE;
                default:
                    return Movement.MOVEARM;
            }
        }
        else
        {
            switch(scoringPos)
            {
                case HATCHL1FORWARDS:
                        return Movement.SAFEZMOVE;
                case HATCHL1BACKWARDS:
                        return Movement.SAFEZMOVE;
                case STOWED:
                        return Movement.SAFEZMOVE;                    
                case CARGOHANDOFF:
                        return Movement.SAFEZMOVE;
                case VERTICALSTOWED:
                        return Movement.SAFEZMOVE;
                default:
                    if(Elevator.elevatorEncoderValue > Constants.elevatorMinRotation)
                        return Movement.MOVEARM;
                    else
                        return Movement.SAFEZMOVE;
            }
        }
    }

    public static boolean threshold(int constant, int currentValue, int threshold)
    {
        if((constant + threshold) > currentValue && (constant - threshold) < currentValue)
		{
			return true;
		}
		else
		{
			return false;
		}
    }

    public RobotPos getRobotPos()
    {
        return new RobotPos(eLevel, armPos);
    }
}