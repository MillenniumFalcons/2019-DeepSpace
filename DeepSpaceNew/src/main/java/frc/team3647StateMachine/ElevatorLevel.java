package frc.team3647StateMachine;

import frc.robot.Constants;

public class ElevatorLevel extends SubsystemAimedState{

    public static final ElevatorLevel MANUAL = new ElevatorLevel();
    public static final ElevatorLevel STOPPED = new ElevatorLevel();
    public static final ElevatorLevel BOTTOM = new ElevatorLevel();
    public static final ElevatorLevel CARGOHANDOFF = new ElevatorLevel(Constants.elevatorCargoHandoff, GameObject.kCargo);
    public static final ElevatorLevel HATCHHANDOFF = new ElevatorLevel(Constants.elevatorHatchHandoff, GameObject.kHatch);
    public static final ElevatorLevel HATCHL2 = new ElevatorLevel(Constants.elevatorHatchL2, GameObject.kHatch);
    public static final ElevatorLevel HATCHL3 = new ElevatorLevel(Constants.elevatorHatchL3, GameObject.kHatch);
    public static final ElevatorLevel CARGOL3 = new ElevatorLevel(Constants.elevatorCargoL3, GameObject.kCargo);
    public static final ElevatorLevel CARGOL2 = new ElevatorLevel(Constants.elevatorCargoL2, GameObject.kCargo);
    public static final ElevatorLevel CARGO1 = new ElevatorLevel(Constants.elevatorCargoL1, GameObject.kCargo);
    public static final ElevatorLevel CARGOSHIP = new ElevatorLevel(Constants.elevatorCargoShip, GameObject.kCargo);
    public static final ElevatorLevel STOWED = new ElevatorLevel(Constants.elevatorStowed, GameObject.kNone);
    public static final ElevatorLevel MINROTATE = new ElevatorLevel(Constants.elevatorMinRotation, GameObject.kNone);
    public static final ElevatorLevel VERTICALSTOWED = new ElevatorLevel(Constants.elevatorMinRotation, GameObject.kNone);
    public static final ElevatorLevel CARGOLOADINGSTATION = new ElevatorLevel(Constants.elevatorCargoLoadingStation, GameObject.kCargo);
    public static final ElevatorLevel BEFORECARGOHANDOFF = new ElevatorLevel(Constants.elevatorBeforeCargoHandoff, GameObject.kCargo);
    public static final ElevatorLevel START = new ElevatorLevel();
    public static final ElevatorLevel NONE = new ElevatorLevel();

    private boolean isAboveMinRotate;

    private ElevatorLevel(int encoderValue, GameObject object){
        super(encoderValue, object);

        isAboveMinRotate = false;
        if(encoderValue >= Constants.elevatorMinRotation){
            isAboveMinRotate = true;
        }
    }
    private ElevatorLevel(){
        super();
        isAboveMinRotate = false;
    }

    /**
     * 
     * @return if elevator level is above a set constant
     */
    public boolean isAboveMinRotate(){
        return isAboveMinRotate;
    }

}