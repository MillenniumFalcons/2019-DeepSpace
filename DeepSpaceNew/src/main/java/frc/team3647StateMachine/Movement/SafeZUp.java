package frc.team3647StateMachine.Movement;

import frc.team3647StateMachine.ArmPosition;

/**
 * moves elevator to next state.
 * if above minRotate, transition to freeMove (both subsystems go wherever they want to)
 */
public class SafeZUp extends SafeMove {
    public void run() {
        if (mElevator.isAboveValue(minRotateToUse.getValue())) {
            mArm.aimedState = currentRobotState.getArmPosition();
            mElevator.aimedState = currentRobotState.getElevatorLevel();
        } else {
            mElevator.aimedState = currentRobotState.getElevatorLevel();
            mArm.aimedState = ArmPosition.STOPPED;
        }
    }
}