package frc.team3647StateMachine;

public class RobotState {

    public static final RobotState MINROTATEFLATBWD = new RobotState(ElevatorLevel.MINROTATE, ArmPosition.HATCHFLATBACKWARDS);
    public static final RobotState MINROTATEFLATFWD = new RobotState(ElevatorLevel.MINROTATE, ArmPosition.HATCHFLATFORWARDS);


    public static final RobotState HATCHL1FORWARDS = new RobotState(ElevatorLevel.BOTTOM, ArmPosition.HATCHFLATFORWARDS);
    public static final RobotState HATCHL1BACKWARDS = new RobotState(ElevatorLevel.BOTTOM, ArmPosition.HATCHFLATBACKWARDS);

    public static final RobotState HATCHL1VISIONFORWARDS = new RobotState(ElevatorLevel.BOTTOM, ArmPosition.FLATVISIONFORWARDS);
    public static final RobotState HATCHL1VISIONBACKWARDS = new RobotState(ElevatorLevel.BOTTOM, ArmPosition.FLATVISIONBACKWARDS);

    public static final RobotState HATCHL2FORWARDS = new RobotState(ElevatorLevel.HATCHL2, ArmPosition.HATCHFLATFORWARDS);
    public static final RobotState HATCHL2BACKWARDS = new RobotState(ElevatorLevel.HATCHL2, ArmPosition.HATCHFLATBACKWARDS);

    public static final RobotState HATCHL3FORWARDS = new RobotState(ElevatorLevel.HATCHL3, ArmPosition.HATCHFLATFORWARDS);
    public static final RobotState HATCHL3BACKWARDS = new RobotState(ElevatorLevel.HATCHL3, ArmPosition.HATCHFLATBACKWARDS);


    public static final RobotState CARGOL1FORWARDS = new RobotState(ElevatorLevel.CARGO1, ArmPosition.CARGOFLATFORWARDS);
    public static final RobotState CARGOL1BACKWARDS = new RobotState(ElevatorLevel.CARGO1, ArmPosition.CARGOFLATBACKWARDS);

    public static final RobotState CARGOSHIPFORWARDS = new RobotState(ElevatorLevel.CARGOSHIP, ArmPosition.CARGOSHIPFORWARDS);
    public static final RobotState CARGOSHIPBACKWARDS = new RobotState(ElevatorLevel.CARGOSHIP, ArmPosition.CARGOSHIPBACKWARDS);

    public static final RobotState CARGOL2FORWARDS = new RobotState(ElevatorLevel.CARGOL2, ArmPosition.CARGOFLATFORWARDS);
    public static final RobotState CARGOL2BACKWARDS = new RobotState(ElevatorLevel.CARGOL2, ArmPosition.CARGOFLATBACKWARDS);

    public static final RobotState CARGOL3FORWARDS = new RobotState(ElevatorLevel.CARGOL3, ArmPosition.CARGOL3FRONT);
    public static final RobotState CARGOL3BACKWARDS = new RobotState(ElevatorLevel.CARGOL3, ArmPosition.CARGOL3BACK);


    public static final RobotState BEFORECARGOHANDOFF = new RobotState(ElevatorLevel.BEFORECARGOHANDOFF, ArmPosition.CARGOHANDOFF);
    public static final RobotState CARGOHANDOFF = new RobotState(ElevatorLevel.CARGOHANDOFF, ArmPosition.CARGOHANDOFF);
    public static final RobotState CARGOLOADINGSTATIONFWD = new RobotState(ElevatorLevel.CARGOLOADINGSTATION, ArmPosition.CARGOFLATFORWARDS);
    public static final RobotState CARGOLOADINGSTATIONBWD = new RobotState(ElevatorLevel.CARGOLOADINGSTATION, ArmPosition.CARGOFLATBACKWARDS);


    public static final RobotState STOWED = new RobotState(ElevatorLevel.STOWED, ArmPosition.STOWED);
    public static final RobotState VERTICALSTOWED = new RobotState(ElevatorLevel.VERTICALSTOWED, ArmPosition.VERTICALSTOWED);

    public static final RobotState REVLIMITSWITCH = new RobotState(ElevatorLevel.MINROTATE, ArmPosition.REVLIMITSWITCH);
    public static final RobotState FWDLIMITSWITCH = new RobotState(ElevatorLevel.MINROTATE, ArmPosition.FWDLIMITSWITCH);

    public static final RobotState NONE = new RobotState(ElevatorLevel.NONE, ArmPosition.NONE);
    public static final RobotState START = new RobotState(ElevatorLevel.NONE, ArmPosition.NONE);
    public static final RobotState BOTTOMSTART = new RobotState(ElevatorLevel.START, ArmPosition.HATCHFLATFORWARDS);

    public static final RobotState CLIMB = new RobotState(ElevatorLevel.MINROTATE, ArmPosition.CLIMB);
    public static final RobotState STOPPED = new RobotState(ElevatorLevel.STOPPED, ArmPosition.STOPPED);

    private ArmPosition mArmPosition;
    private ElevatorLevel mElevatorLevel;
    private boolean isSpecial;

    private RobotState(ElevatorLevel elevatorLevel, ArmPosition armPosition) throws NullPointerException {
        
        /* Intentionally failing code from running if any of the positions are null to not fail during a game */
        if(elevatorLevel == null || armPosition == null) {
            throw new NullPointerException("either elevator level or arm positions were null! ");
        }
        mArmPosition = armPosition;
        mElevatorLevel = elevatorLevel;
        isSpecial = armPosition.isSpecial();
    }

    public ArmPosition getArmPosition() {
        return mArmPosition;
    }

    public ElevatorLevel getElevatorLevel() {
        return mElevatorLevel;
    }

    public boolean isAboveMinRotate() {
        return mElevatorLevel.isAboveMinRotate();
    }

    public boolean isSpecial() {
        return isSpecial;
    }

    public Direction getDirection() {
        return mArmPosition.getDirection();
    }

    public String toString() {
        return "Elevator: " + mElevatorLevel.getValue() + "\nArm: " + mArmPosition.getValue();
    }    
}