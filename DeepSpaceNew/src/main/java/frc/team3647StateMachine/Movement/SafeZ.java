package frc.team3647StateMachine.Movement;

import frc.team3647StateMachine.ArmPosition;

/**
 * moves to minRotate first then moves arm
 */
public class SafeZ extends Movement {
    public void run() {
        if ((mElevator.getEncoderVelocity() > -750) && (mElevator.isAboveValue(minRotate.getValue(), -550)) && (mElevator.getEncoderValue() <= 10000)) {
            mArm.aimedState = currentRobotState.getArmPosition();
        } else {
            mElevator.aimedState = minRotate;
            mArm.aimedState = ArmPosition.STOPPED;
        }
    }
}