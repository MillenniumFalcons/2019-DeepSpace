package frc.team3647autonomous.PathProperties;

public class Side extends PathProperties{

    public static final Side LEFT = new Side("Left");
    public static final Side RIGHT = new Side("Right");

    public Side(String name) {
        super(name);
    }
}