package frc.team3647autonomous.PathProperties;

import frc.team3647StateMachine.RobotState;

public class FieldElement extends PathProperties {

    public static final FieldElement HAB = new FieldElement("Hab", RobotState.START, ScorablePosition.NONE);

    //Scorable elements
    public static final FieldElement LoadingStation = new FieldElement("LoadingStation", RobotState.HATCHL1BACKWARDS, ScorablePosition.NONE);
    public static final FieldElement ROCKETFRONT = new FieldElement("Rocket", RobotState.HATCHL2FORWARDS, ScorablePosition.FRONT);
    public static final FieldElement ROCKETBACK = new FieldElement("Rocket", RobotState.HATCHL2FORWARDS, ScorablePosition.BACK);
    public static final FieldElement cargoShipBay1 = new FieldElement("CargoShip", RobotState.HATCHL1FORWARDS, ScorablePosition.BAY1);
    public static final FieldElement cargoShipBay2 = new FieldElement("CargoShip", RobotState.HATCHL1FORWARDS, ScorablePosition.BAY2);
    public static final FieldElement cargoShipFront = new FieldElement("CargoShip", RobotState.HATCHL1FORWARDS, ScorablePosition.FRONT);

    public RobotState aimedState;
    public ScorablePosition scorablePosition;

    private FieldElement(String name, RobotState aimedState, ScorablePosition scorablePosition) {
        super(name);
        this.aimedState = aimedState;
        this.scorablePosition = scorablePosition;
    }
}
