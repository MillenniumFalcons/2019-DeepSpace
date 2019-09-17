package frc.team3647StateMachine.Movement;

import frc.team3647StateMachine.ArmPosition;
import frc.team3647StateMachine.ElevatorLevel;

/**
 * moves to minRotate first then moves arm
 */
public class SafeZ extends SafeMove {
    public void run() {
        if ((mElevator.getEncoderVelocity() > -750) && (mElevator.isAboveValue(minRotateToUse.getValue())) && (mElevator.getEncoderValue() <= 10000)) {
            mArm.aimedState = currentRobotState.getArmPosition();
        } else {
            mElevator.aimedState = minRotateToUse;
            mArm.aimedState = ArmPosition.STOPPED;
        }
    }
}