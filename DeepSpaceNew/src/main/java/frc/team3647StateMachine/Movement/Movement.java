package frc.team3647StateMachine.Movement;

import frc.team3647StateMachine.RobotState;
import frc.team3647subsystems.Arm;
import frc.team3647subsystems.Elevator;

public abstract class Movement {
    public RobotState currentRobotState = RobotState.NONE;
    protected Elevator mElevator = Elevator.getInstance();
    protected Arm mArm = Arm.getInstance();

    public abstract void run();
    
}