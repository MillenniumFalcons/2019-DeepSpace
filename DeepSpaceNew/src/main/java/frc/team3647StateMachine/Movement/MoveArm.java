package frc.team3647StateMachine.Movement;


public class MoveArm extends Movement {
    public void run() {
        mArm.aimedState = currentRobotState.getArmPosition();
    }
}