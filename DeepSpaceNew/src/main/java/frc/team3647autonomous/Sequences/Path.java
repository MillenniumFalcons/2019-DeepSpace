package frc.team3647autonomous.Sequences;

import frc.robot.Constants;
import frc.team3647autonomous.MotionProfileDirection;
import frc.team3647autonomous.TrajectoryUtil;
import frc.team3647autonomous.PathProperties.Side;
import frc.team3647subsystems.VisionController;
import frc.team3647subsystems.VisionController.VisionMode;
import frc.team3647autonomous.PathProperties.FieldElement;
import jaci.pathfinder.Trajectory;

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

    public double visionPercentage = 1.0;
    public double nextStatePercentage = .5;
    public int visionVelocityConstant = Constants.visionVelocityConstant;

    private Path(Side side, FieldElement startElement, FieldElement endElement, MotionProfileDirection direction,
            double visionPercentage, VisionController pathLimelight, VisionMode visionMode, int visionVelocityConstant) {
        this.trajectoryName = side.asString + startElement.asString + startElement.scorablePosition.asString
                + endElement.asString + endElement.scorablePosition.asString;
        System.out.println("Path Name: " + trajectoryName);
        this.trajectory = TrajectoryUtil.getTrajectoryFromName(this.trajectoryName);
        this.endElement = endElement;
        this.startElement = startElement;
        this.side = side;
        this.direction = direction;
        this.visionPercentage = visionPercentage;
        this.nextStatePercentage = 1 - visionPercentage;
        this.pathLimelight = pathLimelight;
        this.visionMode = visionMode;
        this.visionVelocityConstant = visionVelocityConstant;
    }

    Trajectory getTrajectory() {
        return trajectory;
    }
    /**
     * 
     * @param side which side of the field (left or right)
     * @param startElement the starting element of the path (hab, loading station, cargoship etc.)
     * @param endElement the ending element of the path ^
     * @param direction direction of the path forwards or backwards
     * @param visionPercentage when to start using vision as a percentage from the trajectory ( for rocket and loading station earlier) for cargoShip later (wait for the turn)
     * @return the path
     */
    public static Path createNewPath(Side side, FieldElement startElement, FieldElement endElement, MotionProfileDirection direction, double visionPercentage, int visionVelocityConstant) {

        VisionController pathLimelight = VisionController.limelightClimber;
        VisionMode visionMode = VisionMode.kClosestLvl1;

        if(side == null || startElement == null || endElement == null || direction == null) {
            throw new NullPointerException("NEED TO PROVIDE ALL INFORMATION FOR THE PATH");
        }

        if (direction == MotionProfileDirection.BACKWARD) {
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

        // if(endElement == FieldElement.ROCKETFRONT) {
        //     visionMode = VisionMode.kClosestLvl1;
        // }

        return new Path(side, startElement, endElement, direction, visionPercentage, pathLimelight, visionMode, visionVelocityConstant);
    }
}