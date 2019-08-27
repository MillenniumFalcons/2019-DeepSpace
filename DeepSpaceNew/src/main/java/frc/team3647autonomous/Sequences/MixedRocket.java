package frc.team3647autonomous.Sequences;
import frc.team3647autonomous.MotionProfileDirection;
import frc.team3647autonomous.PathProperties.*;



public class MixedRocket extends Sequence {

    public MixedRocket(Side side) {
        super("MixedRocket"+side.asString, new Path[]{
            new Path(side, FieldElement.HAB, FieldElement.ROCKETBACK, MotionProfileDirection.FORWARD),
            new Path(side, FieldElement.ROCKETBACK, FieldElement.LoadingStation, MotionProfileDirection.BACKWARD),
            new Path(side, FieldElement.LoadingStation, FieldElement.ROCKETFRONT, MotionProfileDirection.FORWARD)
        } );
    }

    public void execute() {
        super.execute();     
    }
}