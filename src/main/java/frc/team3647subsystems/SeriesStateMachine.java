package frc.team3647subsystems;

import edu.wpi.first.wpilibj.RobotState;
import frc.robot.*;
import frc.team3647inputs.*;
import frc.team3647utility.*;

public class SeriesStateMachine
{
    public static RobotPos robotState;
    public static int state = 0;
    
    public static void runSeriesStateMachine(Joysticks controller)
    {
        robotState.initRobotPos();
        Elevator.runElevator();
        Arm.runArm();
    }

    public static void hatchL1Fowards()
    {
        switch(robotState.movementCheck(Elevator.ElevatorLevel.BOTTOM, Arm.ArmPosition.FLATFORWARDS))
        {
            case ARRIVED:
                System.out.println("Arrived at hatchL1Forwards");
                Arm.aimedState = Arm.ArmPosition.FLATFORWARDS;
                Elevator.aimedState = Elevator.ElevatorLevel.BOTTOM;
                break;
            case MOVEELEV:
                Elevator.aimedState = Elevator.ElevatorLevel.BOTTOM;
                break;
            case MOVEARM:
                Arm.aimedState = Arm.ArmPosition.FLATFORWARDS;
            case SAFEZMOVE:
                safetyRotateArm(Arm.ArmPosition.FLATFORWARDS);
            break;
        }
    }

    public static void hatchL1Backwards()
    {

    }

    public static void safetyRotateArm(Arm.ArmPosition pos)
    {
        switch(state)
        {
            case 0:
                if(Elevator.currentState == Elevator.ElevatorLevel.MINROTATE)
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