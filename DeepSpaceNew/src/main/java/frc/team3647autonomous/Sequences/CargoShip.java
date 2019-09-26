package frc.team3647autonomous.Sequences;

import frc.team3647autonomous.MotionProfileDirection;
import frc.team3647autonomous.PathProperties.FieldElement;
import frc.team3647autonomous.PathProperties.Side;

public class CargoShip extends Sequence {

    public CargoShip(Side side) {
        super("CargoShip",
                new Path[] {
                        Path.createNewPath(side, FieldElement.HAB, FieldElement.cargoShipBay2,
                                MotionProfileDirection.FORWARD, .8, 3600),
                        Path.createNewPath(side, FieldElement.cargoShipBay2, FieldElement.LoadingStation,
                                MotionProfileDirection.BACKWARD, .6, 3600),
                        Path.createNewPath(side, FieldElement.LoadingStation, FieldElement.cargoShipBay1,
                                MotionProfileDirection.FORWARD, .82, 3600), }

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