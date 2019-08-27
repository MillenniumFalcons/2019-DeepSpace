package frc.team3647autonomous.Sequences;

import frc.team3647autonomous.MotionProfileDirection;
import frc.team3647autonomous.TrajectoryUtil;
import frc.team3647autonomous.PathProperties.ScorablePosition;
import frc.team3647autonomous.PathProperties.Side;
import frc.team3647subsystems.VisionController;
import frc.team3647subsystems.VisionController.VisionMode;
import frc.team3647autonomous.PathProperties.FieldElement;
import frc.team3647autonomous.PathProperties.PathProperties;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;

/**
 * A path can have up to one trajectory also gives the sequence percentage
 * points for changing state machine and starting vision
 */
public class Path {

    private Trajectory trajectory;
    private String trajectoryName;

    public FieldElement endElement;
    public FieldElement startElement;
    public Side side;
    public MotionProfileDirection direction;
    public VisionMode visionMode;
    public VisionController pathLimelight;

    public Path(Side side, FieldElement startElement, FieldElement endElement, MotionProfileDirection direction) {
        this.trajectoryName = side.asString + startElement.asString + startElement.scorablePosition.asString
                + endElement.asString + endElement.scorablePosition.asString;
        this.trajectory = TrajectoryUtil.getTrajectoryFromName(this.trajectoryName);
        this.endElement = endElement;
        this.startElement = startElement;
        this.side = side;
        this.direction = direction;

        pathLimelight = VisionController.limelightClimber;
        visionMode = VisionMode.kClosestLvl1;

        if (this.direction == MotionProfileDirection.BACKWARD) {
            pathLimelight = VisionController.limelightFourbar;
        }
        
        if (side == Side.LEFT) {
            if (endElement == FieldElement.ROCKETBACK || endElement == FieldElement.cargoShipBay1) {
                visionMode = VisionMode.kRight;
            } else if (endElement == FieldElement.ROCKETFRONT || endElement == FieldElement.cargoShipFront) {
                visionMode = VisionMode.kLeft;
            }
        } else {
            if (endElement == FieldElement.ROCKETBACK || endElement == FieldElement.cargoShipBay1) {
                visionMode = VisionMode.kRight;
            } else if (endElement == FieldElement.ROCKETFRONT || endElement == FieldElement.cargoShipFront) {
                visionMode = VisionMode.kLeft;
            }

        }
    }

    Trajectory getTrajectory() {
        return trajectory;
    }
}