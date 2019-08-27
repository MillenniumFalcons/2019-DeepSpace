package frc.team3647autonomous.Sequences;

import jaci.pathfinder.Trajectory.Segment;
import frc.team3647subsystems.Drivetrain;
import frc.team3647subsystems.VisionController;
import frc.team3647subsystems.VisionController.VisionMode;
import frc.team3647utility.Units;
import frc.team3647autonomous.RamseteFollower;
import frc.team3647autonomous.PathProperties.Side;
import frc.team3647autonomous.PathProperties.Side;
import frc.team3647autonomous.DriveSignal;
import frc.team3647autonomous.MotionProfileDirection;
import frc.team3647autonomous.Odometry;

public abstract class Sequence {

    private RamseteFollower ramseteFollower;
    private Path[] pathArr;
    private Drivetrain mDrivetrain = Drivetrain.getInstance();
    private Side side;
    protected int currentPathIndex;
    // private
    private String name;

    protected boolean initialized = false;
    protected VisionController mVision = VisionController.limelightClimber;
    protected DriveSignal driveSignal;
    protected Segment currentSegment;
    protected double rightSpeed;
    protected double leftSpeed;
    protected double linearVelocity;

    public void ramsetePeriodic() {
        driveSignal = ramseteFollower.getNextDriveSignal();
        currentSegment = ramseteFollower.currentSegment();
        rightSpeed = Units.metersToEncoderTicks(driveSignal.getRight() / 10);
        leftSpeed = Units.metersToEncoderTicks(driveSignal.getLeft() / 10);
        linearVelocity = Units.metersToEncoderTicks(ramseteFollower.linearVelocity / 10);
    }

    public Sequence(String name, Path[] pathArr) throws IndexOutOfBoundsException {
        if (pathArr.length != 3) {
            throw new IndexOutOfBoundsException("Array must be 3 elements exactly");
        } else {
            this.pathArr = pathArr;
        }
        this.name = name;
        this.side = pathArr[0].side;
        initialized = false;
    }

    /**
     * 
     * @param direction either FORWARD or BACKWARD
     * @param index     the index of the path, 0,1,2
     */
    public void init(int index) {
        currentPathIndex = index;

        mDrivetrain.configDefaultPIDF();
        driveSignal = new DriveSignal();
        mVision = pathArr[index].pathLimelight;
        mVision.set(pathArr[index].visionMode);

        if (index == 0) {
            Odometry.getInstance().setInitialOdometry(pathArr[index].getTrajectory());
            Odometry.getInstance().odometryInit();
        }
        ramseteFollower = new RamseteFollower(pathArr[index].getTrajectory(), pathArr[index].direction);

    }

    public void execute() {
        ramsetePeriodic();
        if (!initialized) {
            init(currentPathIndex);
            initialized = true;
        }
    }

    public String getName() {
        return this.name;
    }

    public String toString() {
        return getName();
    }

}