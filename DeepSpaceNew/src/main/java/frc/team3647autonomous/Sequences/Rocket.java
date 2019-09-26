package frc.team3647autonomous.Sequences;

import frc.robot.Constants;
import frc.team3647autonomous.MotionProfileDirection;
import frc.team3647autonomous.PathProperties.FieldElement;
import frc.team3647autonomous.PathProperties.Side;

public class Rocket extends Sequence {

    public Rocket(Side side) {
        super("Rocket",
                new Path[] {
                        Path.createNewPath(side, FieldElement.HAB, FieldElement.ROCKETFRONT,
                                MotionProfileDirection.FORWARD, .4, 2700),
                        Path.createNewPath(side, FieldElement.ROCKETFRONT, FieldElement.LoadingStation,
                                MotionProfileDirection.BACKWARD, .25, 4000),
                        Path.createNewPath(side, FieldElement.LoadingStation, FieldElement.ROCKETFRONT2,
                                MotionProfileDirection.FORWARD, .3, 4000), }

        );
    }

    @Override
    public void init(int index) {
        super.init(index);
    }

    @Override
    public void execute() {
        super.execute();
    }

}