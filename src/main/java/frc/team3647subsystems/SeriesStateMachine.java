package frc.team3647subsystems;

import edu.wpi.first.wpilibj.RobotState;
import frc.robot.*;
import frc.team3647inputs.*;
import frc.team3647utility.*;

public class SeriesStateMachine
{
    public static RobotPos robotState;
    public static RobotPos hatchL1Forwards, hatchL1Backwards, hatchL2forwards, hatchL2Backwards;
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
        CARGOL3FORWARDS, // Move elevator first
        CARGOL3BACKWARDS, // move elevator first
        HATCHHANDOFF, //ARM HIT
        CARGOHANDOFF, //ARM HIT, make sure cargo ground intake is deployed
        STOWED, //ARM HIT, Hatch intake is stowed
        VERTICALSTOWED //ARM HIT
    }

    public static void seriesStateMachineInitialization()
    {
        robotState = new RobotPos(Elevator.currentState, Arm.currentState);   
        hatchL1Backwards = new RobotPos(Elevator.ElevatorLevel.BOTTOM, Arm.ArmPosition.FLATBACKWARDS);        
        hatchL1Forwards = new RobotPos(Elevator.ElevatorLevel.BOTTOM, Arm.ArmPosition.FLATFORWARDS);
        
    }

    
    public static void runSeriesStateMachine(Joysticks controller)
    {
        if(Arm.currentState != null && Elevator.currentState != null)
            robotState.setRobotPos(Arm.currentState, Elevator.currentState);
        
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
                break;
            case SAFEZMOVE:
                System.out.println("Running Safe Z");
                safetyRotateArm(hatchL1Forwards.armPos);
                break;
            case FREEMOVE:
                System.out.println("Running Freemove");
                Arm.aimedState = hatchL1Forwards.armPos;
                Elevator.aimedState = hatchL1Forwards.eLevel;
                break;
        }
    }

    public static void hatchL1Backwards()
    {
        switch(robotState.movementCheck(ScoringPosition.HATCHL1BACKWARDS, hatchL1Backwards))
        {
            case ARRIVED:
                System.out.println("Arrived at hatchL1Backwards");
                Arm.aimedState = hatchL1Backwards.armPos;
                Elevator.aimedState = hatchL1Backwards.eLevel;
                break;
            case MOVEELEV:
                System.out.println("Moving Elevator");
                Elevator.aimedState = hatchL1Backwards.eLevel;
                break;
            case MOVEARM:
                System.out.println("Moving Arm");
                Arm.aimedState = hatchL1Backwards.armPos;
                break;
            case SAFEZMOVE:
                System.out.println("Running Safe Z");
                safetyRotateArm(hatchL1Backwards.armPos);
                break;
            case FREEMOVE:
                System.out.println("Running free move");
                Arm.aimedState = hatchL1Backwards.armPos;
                Elevator.aimedState = hatchL1Backwards.eLevel;
                break;
        }
    }

    public static void safetyRotateArm(Arm.ArmPosition pos)
    {
        // switch(state)
        // {
        //     case 0:
                if(Elevator.elevatorEncoderValue >= Constants.elevatorMinRotation-100)
                {
                    state = 1;
                    Arm.aimedState = pos;
                }
                else
                {
                    Elevator.aimedState = Elevator.ElevatorLevel.MINROTATE;
                    Arm.aimedState = null;
                //}
                //break;
            //case 1:
                    //Arm.aimedState = pos;
                }
                //break;
       // }
    }
}