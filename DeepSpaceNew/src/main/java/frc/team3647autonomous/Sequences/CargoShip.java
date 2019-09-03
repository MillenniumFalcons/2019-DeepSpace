package frc.team3647autonomous.Sequences;

import frc.team3647autonomous.MotionProfileDirection;
import frc.team3647autonomous.PathProperties.FieldElement;
import frc.team3647autonomous.PathProperties.Side;

public class CargoShip extends Sequence {

    public CargoShip(Side side) {
        super("CargoShip",
            new Path[] {
                new Path(side, FieldElement.HAB, FieldElement.cargoShipBay2, MotionProfileDirection.FORWARD),
                new Path(side, FieldElement.cargoShipBay2, FieldElement.LoadingStation, MotionProfileDirection.BACKWARD),
                new Path(side, FieldElement.LoadingStation, FieldElement.cargoShipBay1, MotionProfileDirection.FORWARD),
            }

        );
    }

    @Override
    public void init(int index) {
        super.init(index);
    }

}