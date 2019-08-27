package frc.team3647StateMachine.Movement;


public class MoveElevator extends Movement {
    public void run() {
        mElevator.aimedState = currentRobotState.getElevatorLevel();
    }
}