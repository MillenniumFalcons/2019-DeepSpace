package frc.team3647StateMachine;

/**
 * predetermined positions for arm and elevator, or whatever the robot can be doing at a time
 */
public class RobotState {

    public static final RobotState MINROTATEFLATBWD = new RobotState(ElevatorLevel.MINROTATE, ArmPosition.HATCHFLATBACKWARDS);
    public static final RobotState MINROTATEFLATFWD = new RobotState(ElevatorLevel.MINROTATE, ArmPosition.HATCHFLATFORWARDS);


    public static final RobotState HATCHL1FORWARDS = new RobotState(ElevatorLevel.BOTTOM, ArmPosition.HATCHFLATFORWARDS);
    public static final RobotState HATCHL1BACKWARDS = new RobotState(ElevatorLevel.BOTTOM, ArmPosition.HATCHFLATBACKWARDS);

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
    public static final RobotState VERTICALSTOWED = new RobotState(ElevatorLevel.BOTTOM, ArmPosition.VERTICALSTOWED);

    public static final RobotState REVLIMITSWITCH = new RobotState(ElevatorLevel.MINROTATE, ArmPosition.REVLIMITSWITCH);
    public static final RobotState FWDLIMITSWITCH = new RobotState(ElevatorLevel.MINROTATE, ArmPosition.FWDLIMITSWITCH);

    public static final RobotState NONE = new RobotState(ElevatorLevel.NONE, ArmPosition.NONE);
    public static final RobotState START = new RobotState(ElevatorLevel.NONE, ArmPosition.NONE);
    public static final RobotState BOTTOMSTART = new RobotState(ElevatorLevel.BOTTOM, ArmPosition.HATCHFLATFORWARDS);

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

    public RobotState getStateAtMinRotate(RobotState currentState) {
        if(currentState.isAboveMinRotate()) {
            return currentState;
        }
        return new RobotState(ElevatorLevel.MINROTATE, currentState.getArmPosition());
    }

    public ArmPosition getArmPosition() {
        return mArmPosition;
    }

    public ElevatorLevel getElevatorLevel() {
        return mElevatorLevel;
    }

    /**
     * @return is the elevator level of this aimed state above minrotate
     */
    public boolean isAboveMinRotate() {
        return mElevatorLevel.isAboveOtherLevel(mArmPosition.getApplicableMinRotate());
    }

    /**
     * 
     * @return if the state has to use something other than motion magic to get to it, or the default run method
     */
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