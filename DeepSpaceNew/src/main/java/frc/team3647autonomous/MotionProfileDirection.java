package frc.team3647autonomous;

public enum MotionProfileDirection 
{
    BACKWARD("BACKWARD"), 
    FORWARD("FORWARD");
    public String asString;
    MotionProfileDirection(String asString) {
        this.asString = asString;
    }
}
