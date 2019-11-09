package frc.team3647StateMachine;

import frc.robot.Constants;

public class ArmPosition extends SubsystemAimedState {

    public static final ArmPosition FWDLIMITSWITCH = new ArmPosition();
    public static final ArmPosition REVLIMITSWITCH = new ArmPosition();
    /**
     * Score cargo backwards on rocket
     */
    public static final ArmPosition CARGOFLATBACKWARDS= new ArmPosition(Constants.armSRXCargoFlatBackwards, GameObject.kCargo);
    /**
     * score cargo forwards on rocket
     */
    public static final ArmPosition CARGOFLATFORWARDS = new ArmPosition(Constants.armSRXCargoFlatForwards, GameObject.kCargo);
    public static final ArmPosition HATCHFLATFORWARDS = new ArmPosition(Constants.armSRXFlatForwards, GameObject.kHatch);
    public static final ArmPosition HATCHFLATBACKWARDS = new ArmPosition(Constants.armSRXFlatBackwards, GameObject.kHatch);
    public static final ArmPosition HATCHFLATBACKWARDSRESET = new ArmPosition(Constants.armSRXFlatBackwardsReset, GameObject.kHatch);
    public static final ArmPosition HATCHFLATFORWARDSRESET = new ArmPosition(Constants.armSRXFlatForwardsReset, GameObject.kHatch);
    public static final ArmPosition HATCHFLATBACKWARDSAUTO = new ArmPosition(Constants.armSRXAutoFlatBackwards, GameObject.kHatch);

    public static final ArmPosition HATCHFLATFORWARDSL3 = new ArmPosition(Constants.armSRXLevel3FlatForwards, GameObject.kHatch);
    public static final ArmPosition HATCHFLATBACKWARDSL3 = new ArmPosition(Constants.armSRXLevel3FlatBackwards, GameObject.kHatch);
    
    public static final ArmPosition CARGOL3FRONT = new ArmPosition(Constants.armSRXCargoL3Front, GameObject.kCargo);
    public static final ArmPosition CARGOL3BACK = new ArmPosition(Constants.armSRXCargoL3Back, GameObject.kCargo);
    public static final ArmPosition HATCHHANDOFF = new ArmPosition(Constants.armSRXHatchHandoff, GameObject.kHatch);
    /**
     * @deprecated use vertical stowed after arm change
     */
    public static final ArmPosition STOWED = new ArmPosition(Constants.armSRXStowed);
    public static final ArmPosition VERTICALSTOWED = new ArmPosition(Constants.armSRXVerticalStowed, GameObject.kHatch);
    /**
     * for when cargo moves between ground intake and the ball shooter on the arm
     */
    public static final ArmPosition CARGOHANDOFF = new ArmPosition(Constants.armSRXCargoHandoff, GameObject.kCargo);
    public static final ArmPosition MANUAL = new ArmPosition();
    public static final ArmPosition STOPPED = new ArmPosition();
    /**
     * @deprecated no climb
     */
    public static final ArmPosition CLIMB = new ArmPosition(Constants.armSRXClimb);
    public static final ArmPosition CARGOSHIPBACKWARDS = new ArmPosition(Constants.armSRXCargoShipBack, GameObject.kCargo);
    public static final ArmPosition CARGOSHIPFORWARDS = new ArmPosition(Constants.armSRXCargoShipFront, GameObject.kCargo);
    public static final ArmPosition REVLIMSWITCHSTART = new ArmPosition();
    /**
     * so that we don't have to use a null pointer
     */
    public static final ArmPosition NONE = new ArmPosition();

    private Direction dir;
    private ElevatorLevel minRotateToUse;

    private ArmPosition(int encoderValue, GameObject object){
        super(encoderValue, object);

        if(encoderValue >= Constants.armSRXVerticalStowed){
            dir = Direction.kBackwards;
        }else{
            dir = Direction.kForwards;
        }


        if(encoderValue < 4000) {
            this.minRotateToUse = ElevatorLevel.MINROTATEFRONT;
        } else if (encoderValue > 22500) {
            this.minRotateToUse = ElevatorLevel.MINROTATEBACK;
        } else {
            this.minRotateToUse = ElevatorLevel.MINROTATE;
        }
        
        setSpecial(this.encoderValue == -1);
    }

    private ArmPosition(int encoderValue) {
        this(encoderValue, GameObject.kNone);
    }
    
    private ArmPosition(){
        this(-1);
    }

    public Direction getDirection(){
        return dir;
    }

    public ElevatorLevel getApplicableMinRotate() {
        return minRotateToUse;
    }
}