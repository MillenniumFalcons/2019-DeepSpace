package frc.team3647StateMachine.Movement;

import frc.team3647StateMachine.ArmPosition;
import frc.team3647StateMachine.ElevatorLevel;

public class SafeZ extends Movement {
    public void run() {
        if ((mElevator.getEncoderVelocity() > -750) && (mElevator.isAboveMinRotate(-550)) && (mElevator.getEncoderValue() <= 30000)) {
            mArm.aimedState = currentRobotState.getArmPosition();
        } else {
            mElevator.aimedState = ElevatorLevel.MINROTATE;
            mArm.aimedState = ArmPosition.STOPPED;
        }
    }
}