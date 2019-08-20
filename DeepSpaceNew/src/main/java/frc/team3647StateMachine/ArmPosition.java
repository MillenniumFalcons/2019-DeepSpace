package frc.team3647StateMachine;

import frc.robot.Constants;

public class ArmPosition extends SubsystemAimedState{

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
    private ArmPosition(){
        super();
    }

    public Direction getDirection(){
        return dir;
    }


    public static final ArmPosition FWDLIMITSWITCH = new ArmPosition();
    public static final ArmPosition REVLIMITSWITCH = new ArmPosition();
    public static final ArmPosition FLATFORWARDS = new ArmPosition(Constants.armSRXFlatForwards, GameObject.kAgnostic);
    public static final ArmPosition FLATVISIONFORWARDS = new ArmPosition(Constants.armSRXForwardVision, GameObject.kNone);
    public static final ArmPosition FLATVISIONBACKWARDS = new ArmPosition(Constants.armSRXBackwardVision, GameObject.kNone);
    public static final ArmPosition FLATBACKWARDS = new ArmPosition(Constants.armSRXFlatBackwards, GameObject.kAgnostic);
    public static final ArmPosition CARGOL3FRONT = new ArmPosition(Constants.armSRXCargoL3Front, GameObject.kCargo);
    public static final ArmPosition CARGOL3BACK = new ArmPosition(Constants.armSRXCargoL3Back, GameObject.kCargo);
    public static final ArmPosition HATCHHANDOFF = new ArmPosition(Constants.armSRXHatchHandoff, GameObject.kHatch);
    public static final ArmPosition STOWED = new ArmPosition(Constants.armSRXStowed, GameObject.kNone);
    public static final ArmPosition VERTICALSTOWED = new ArmPosition(Constants.armSRXVerticalStowed, GameObject.kNone);
    public static final ArmPosition CARGOHANDOFF = new ArmPosition(Constants.armSRXCargoHandoff, GameObject.kCargo);
    public static final ArmPosition MANUAL = new ArmPosition();
    public static final ArmPosition STOPPED = new ArmPosition();
    public static final ArmPosition CLIMB = new ArmPosition(Constants.armSRXClimb, GameObject.kNone);
    public static final ArmPosition VISIONF = new ArmPosition(Constants.armSRXForwardVision, GameObject.kNone);
    public static final ArmPosition VISIONB = new ArmPosition(Constants.armSRXBackwardVision, GameObject.kNone);
    public static final ArmPosition CARGOSHIPBACKWARDS = new ArmPosition(Constants.armSRXCargoShipBack, GameObject.kCargo);
    public static final ArmPosition CARGOSHIPFORWARDS = new ArmPosition(Constants.armSRXCargoShipFront, GameObject.kCargo);
    public static final ArmPosition REVLIMSWITCHSTART = new ArmPosition();
    public static final ArmPosition NONE = new ArmPosition();
}