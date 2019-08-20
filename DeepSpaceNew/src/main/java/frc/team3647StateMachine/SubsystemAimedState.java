package frc.team3647StateMachine;

public abstract class SubsystemAimedState{

    protected int encoderValue;
    private boolean isSpecial;
    private GameObject object;

    protected SubsystemAimedState(int encoderValue, GameObject object){
        this.object = object;
        this.encoderValue = encoderValue;
        isSpecial = false;
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

    public GameObject getObject() {
        return this.object;
    }
}