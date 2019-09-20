package frc.team3647StateMachine.Movement;

import frc.team3647StateMachine.ElevatorLevel;

/**
 * moves to elevator minrotate and arm to aimedRobotState specified position
 */
public class SafeZDown extends Movement {
    public void run() {
        mElevator.aimedState = minRotate;
        mArm.aimedState = currentRobotState.getArmPosition();
    }
}