package frc.team3647utility;

import frc.team3647subsystems.*;
import frc.robot.*;

import edu.wpi.first.wpilibj.Notifier;

public class RobotPos
{
    public Elevator.ElevatorLevel eLevel;
    public Arm.ArmPosition armPos;

    public enum movement
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
            armPos = Arm.currentState;
        });
        robotPosThread.startPeriodic(0.01);
    }

    public movement movementCheck(Elevator.ElevatorLevel elevatorDesired, Arm.ArmPosition armDesired)
    {
        if(elevatorDesired == eLevel && armDesired == armPos)
        {
            return movement.ARRIVED;
        }
        else if(elevatorDesired != eLevel && armDesired == armPos)
        {
            return movement.MOVEELEV;
        }
        else if(elevatorDesired == eLevel && armDesired != armPos) //NEED TO SET A THREHSOLD HERE FOR WHEN ARM CAN MOVE WITHOUT SAFEZ
        {
            return movement.MOVEARM;
        }
        else
        {
            return movement.SAFEZMOVE;
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

    public RobotPos getPos()
    {
        return new RobotPos(Elevator.currentState, Arm.currentState);
    }
}