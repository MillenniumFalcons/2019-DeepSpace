package frc.team3647autonomous.PathProperties;

public class ScorablePosition extends PathProperties {

    public static final ScorablePosition BACK = new ScorablePosition("Back");
    public static final ScorablePosition FRONT = new ScorablePosition("Front");
    public static final ScorablePosition BAY1 = new ScorablePosition("Bay1");
    public static final ScorablePosition BAY2 = new ScorablePosition("Bay2");
    public static final ScorablePosition NONE = new ScorablePosition("");

    public ScorablePosition(String name) {
        super(name);
    }
}