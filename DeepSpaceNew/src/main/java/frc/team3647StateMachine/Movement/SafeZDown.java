package frc.team3647StateMachine.Movement;

import frc.team3647StateMachine.ElevatorLevel;

public class SafeZDown extends Movement {
    public void run() {
        mElevator.aimedState = ElevatorLevel.MINROTATE;
        mArm.aimedState = currentRobotState.getArmPosition();
    }
}