package frc.team3647StateMachine.Movement;

public class FreeMove extends Movement {
    public void run() {
        mArm.aimedState = currentRobotState.getArmPosition();
        mElevator.aimedState = currentRobotState.getElevatorLevel();
    }
}