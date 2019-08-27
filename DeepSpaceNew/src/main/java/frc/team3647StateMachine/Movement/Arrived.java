package frc.team3647StateMachine.Movement;

import frc.team3647StateMachine.RobotState;

public class Arrived extends Movement {
    private Runnable cargoHandoffMethod;

    public Arrived(Runnable cargoHandoffMethod) {
        this.cargoHandoffMethod = cargoHandoffMethod;
    } 
    public void run() {
        mElevator.aimedState = currentRobotState.getElevatorLevel();
        mArm.aimedState = currentRobotState.getArmPosition();
        if(currentRobotState.equals(RobotState.CARGOHANDOFF)) {
            cargoHandoffMethod.run();
        }
    }
}