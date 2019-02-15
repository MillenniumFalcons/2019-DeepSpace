package frc.team3647subsystems;

import edu.wpi.first.wpilibj.RobotState;
import frc.robot.*;
import frc.team3647inputs.*;
import frc.team3647utility.*;

public class SeriesStateMachine
{
    public static RobotPos robotState;
    public static RobotPos hatchL1Forwards, hatchL1Backwards, hatchL2Forwards, HatchL2Backwards;
    public static int state = 0;

    public enum ScoringPosition
    {
        HATCHL1FORWARDS, //ARM HIT
        HATCHL1BACKWARDS, //ARM HIT
        HATCHL2FORWARDS,
        HATCHL2BACKWARDS,
        HATCHL3FORWARDS,
        HATCHL3BACKWARDS,
        CARGOSHIPFORWARD,
        CARGOSHIPBACKWARD,
        CARGOL2FORWARDS,
        CARGOL2BACKWARDS,
        CARGOL3FORWARDS,
        CARGOL3BACKWARDS,
        HATCHHANDOFF, //ARM HIT
        CARGOHANDOFF, //ARM HIT
        STOWED, //ARM HIT
        VERTICALSTOWED //ARM HIT
    }
    
    public static void runSeriesStateMachine(Joysticks controller)
    {
        robotState.initRobotPos();
        Elevator.runElevator();
        Arm.runArm();

        if(controller.buttonB)
        {
            hatchL1Fowards();
        }
        else if(controller.buttonX)
        {
            hatchL1Backwards();
        }
    }

    public static void hatchL1Fowards()
    {
        hatchL1Forwards = new RobotPos(Elevator.ElevatorLevel.BOTTOM, Arm.ArmPosition.FLATFORWARDS);
        switch(robotState.movementCheck(ScoringPosition.HATCHL1FORWARDS, hatchL1Forwards))
        {
            case ARRIVED:
                System.out.println("Arrived at hatchL1Forwards");
                Arm.aimedState = hatchL1Forwards.armPos;
                Elevator.aimedState = hatchL1Forwards.eLevel;
                break;
            case MOVEELEV:
                System.out.println("Moving Elevator");
                Elevator.aimedState = hatchL1Forwards.eLevel;
                break;
            case MOVEARM:
                System.out.println("Moving Arm");
                Arm.aimedState = hatchL1Forwards.armPos;
            case SAFEZMOVE:
                System.out.println("Running Safe Z");
                safetyRotateArm(hatchL1Forwards.armPos);
            break;
        }
    }

    public static void hatchL1Backwards()
    {
        hatchL1Backwards = new RobotPos(Elevator.ElevatorLevel.BOTTOM, Arm.ArmPosition.FLATBACKWARDS);
        switch(robotState.movementCheck(ScoringPosition.HATCHL1FORWARDS, hatchL2Forwards))
        {
            case ARRIVED:
                System.out.println("Arrived at hatchL1Backwards");
                Arm.aimedState = hatchL2Forwards.armPos;
                Elevator.aimedState = hatchL2Forwards.eLevel;
                break;
            case MOVEELEV:
                System.out.println("Moving Elevator");
                Elevator.aimedState = hatchL2Forwards.eLevel;
                break;
            case MOVEARM:
                System.out.println("Moving Arm");
                Arm.aimedState = hatchL2Forwards.armPos;
            case SAFEZMOVE:
                System.out.println("Running Safe Z");
                safetyRotateArm(hatchL2Forwards.armPos);
            break;
        }
    }

    public static void safetyRotateArm(Arm.ArmPosition pos)
    {
        switch(state)
        {
            case 0:
                if(robotState.eLevel == Elevator.ElevatorLevel.MINROTATE)
                {
                    state = 1;
                }
                else
                {
                    Elevator.aimedState = Elevator.ElevatorLevel.MINROTATE;
                }
                break;
            case 1:
                    Arm.aimedState = pos;
                break;
        }
    }
}