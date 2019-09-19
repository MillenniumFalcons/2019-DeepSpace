package frc.team3647StateMachine.Movement;

import frc.team3647StateMachine.ElevatorLevel;
import frc.team3647StateMachine.RobotState;
import frc.team3647subsystems.Arm;
import frc.team3647subsystems.Elevator;

public abstract class Movement {
    public RobotState currentRobotState = RobotState.NONE;
    public ElevatorLevel minRotate = ElevatorLevel.MINROTATEBACK;
    protected Elevator mElevator = Elevator.getInstance();
    protected Arm mArm = Arm.getInstance();
    /**
     * specifies what happens when movementCheck() tells you to do this movement
     */
    public abstract void run();
    
}