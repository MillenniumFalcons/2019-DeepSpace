package frc.team3647StateMachine;

import frc.robot.Constants;

public class ArmPosition extends SubsystemAimedState {

    public static final ArmPosition FWDLIMITSWITCH = new ArmPosition();
    public static final ArmPosition REVLIMITSWITCH = new ArmPosition();
    public static final ArmPosition CARGOFLATBACKWARDS= new ArmPosition(Constants.armSRXCargoFlatForwards, GameObject.kCargo);
    public static final ArmPosition CARGOFLATFORWARDS = new ArmPosition(Constants.armSRXCargoFlatBackwards, GameObject.kCargo);
    public static final ArmPosition HATCHFLATFORWARDS = new ArmPosition(Constants.armSRXFlatForwards, GameObject.kHatch);
    public static final ArmPosition HATCHFLATBACKWARDS = new ArmPosition(Constants.armSRXFlatBackwards, GameObject.kHatch);
    public static final ArmPosition CARGOL3FRONT = new ArmPosition(Constants.armSRXCargoL3Front, GameObject.kCargo);
    public static final ArmPosition CARGOL3BACK = new ArmPosition(Constants.armSRXCargoL3Back, GameObject.kCargo);
    public static final ArmPosition HATCHHANDOFF = new ArmPosition(Constants.armSRXHatchHandoff, GameObject.kHatch);
    public static final ArmPosition STOWED = new ArmPosition(Constants.armSRXStowed);
    public static final ArmPosition VERTICALSTOWED = new ArmPosition(Constants.armSRXVerticalStowed);
    public static final ArmPosition CARGOHANDOFF = new ArmPosition(Constants.armSRXCargoHandoff, GameObject.kCargo);
    public static final ArmPosition MANUAL = new ArmPosition();
    public static final ArmPosition STOPPED = new ArmPosition();
    public static final ArmPosition CLIMB = new ArmPosition(Constants.armSRXClimb);
    public static final ArmPosition VISIONF = new ArmPosition(Constants.armSRXForwardVision, GameObject.kHatch);
    public static final ArmPosition VISIONB = new ArmPosition(Constants.armSRXBackwardVision, GameObject.kHatch);
    public static final ArmPosition CARGOSHIPBACKWARDS = new ArmPosition(Constants.armSRXCargoShipBack, GameObject.kCargo);
    public static final ArmPosition CARGOSHIPFORWARDS = new ArmPosition(Constants.armSRXCargoShipFront, GameObject.kCargo);
    public static final ArmPosition REVLIMSWITCHSTART = new ArmPosition();
    public static final ArmPosition FLATVISIONFORWARDS = new ArmPosition(Constants.armSRXForwardVision, GameObject.kHatch);
    public static final ArmPosition FLATVISIONBACKWARDS = new ArmPosition(Constants.armSRXBackwardVision, GameObject.kHatch);
    public static final ArmPosition NONE = new ArmPosition();

    private Direction dir;

    private ArmPosition(int encoderValue, GameObject object){
        super(encoderValue, object);

        if(GameObject.kCargo.equals(object)){
            if(encoderValue >= Constants.armSRXVerticalStowed){
                dir = Direction.kForwards;
            }else{
                dir = Direction.kBackwards;
            }
        }else { //either hatch, or nothing
            if(encoderValue >= Constants.armSRXVerticalStowed){
                dir = Direction.kBackwards;
            }else{
                dir = Direction.kForwards;
            }
        }
    }

    private ArmPosition(int encoderValue) {
        this(encoderValue, GameObject.kNone);
    }
    private ArmPosition(){
        super();
    }

    public Direction getDirection(){
        return dir;
    }
}