package frc.team3647StateMachine;

public abstract class SubsystemAimedState{

    protected int encoderValue;
    private boolean isSpecial;
    private GameObject object;

    protected SubsystemAimedState(int encoderValue, GameObject object) throws NullPointerException{
        this.object = object;
        if(this.object == null) {
            throw new NullPointerException("Game object is null!");
        }
        this.encoderValue = encoderValue;
        isSpecial = false;
    }

    protected SubsystemAimedState(int encoderValue) {
        this(encoderValue, GameObject.kNone);
    }

    protected SubsystemAimedState(){
        this.object = GameObject.kNone;
        this.encoderValue = -1;
        isSpecial = true;
    }

    /**
     * 
     * @return encoder value of the state
     */
    public int getValue(){
        return encoderValue;
    }

    /**
     * state must be reached without motion magic or does not require movement
     */
    public boolean isSpecial(){
        return isSpecial;
    }

    /**
     * @return the object the robot will have at this position
     */
    public GameObject getObject() {
        return this.object;
    }
}