package frc.team3647StateMachine.Movement;

/**
 * both subsystems go wherever
 */
public class FreeMove extends Movement {
    public void run() {
        mArm.aimedState = currentRobotState.getArmPosition();
        mElevator.aimedState = currentRobotState.getElevatorLevel();
    }
}