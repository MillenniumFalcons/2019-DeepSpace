package frc.team3647autonomous.Sequences;

import jaci.pathfinder.Trajectory.Segment;
import frc.team3647subsystems.Drivetrain;
import frc.team3647subsystems.HatchGrabber;
import frc.team3647subsystems.VisionController;
import frc.team3647subsystems.VisionController.VisionMode;
import frc.team3647utility.Timer;
import frc.team3647utility.Units;
import frc.team3647autonomous.RamseteFollower;
import frc.team3647autonomous.PathProperties.Side;
import frc.team3647autonomous.PathProperties.Side;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.Constants;
import frc.team3647StateMachine.SeriesStateMachine;
import frc.team3647autonomous.DriveSignal;
import frc.team3647autonomous.MotionProfileDirection;
import frc.team3647autonomous.Odometry;

public abstract class Sequence {

    private RamseteFollower ramseteFollower;
    private int visionVelocityMultiplier = Constants.visionVelocityConstant;
    private Path[] pathArr;
    private Drivetrain mDrivetrain = Drivetrain.getInstance();
    private HatchGrabber mHatchGrabber = HatchGrabber.getInstance();
    private SeriesStateMachine stateMachine = SeriesStateMachine.getInstance();
    private Side side;
    private MotionProfileDirection currentDirection;
    public Notifier drivetrainBackwardsNotifier = new Notifier(() -> {
        log("drivetrain notifier started");
        mDrivetrain.setAutoVelocity(-500, -500);
        Timer.delay(3);
        mDrivetrain.stop();
    });;
    protected int nextPathIndex = 0;
    protected int currentIndex = -1;
    // private
    private String name;

    protected boolean initialized = false;
    protected VisionController mVision = VisionController.limelightClimber;
    protected DriveSignal driveSignal;
    protected Segment currentSegment;
    protected double rightSpeed;
    protected double leftSpeed;
    protected double linearVelocity;
    protected boolean useVision = false;
    protected boolean currentPathFinished = false;
    protected boolean goToNextState;
    protected boolean currentPathIsForwards = true;
    private boolean sequenceFinished;

    public void ramsetePeriodic() {
        log("from ramsete periodic");
        driveSignal = ramseteFollower.getNextDriveSignal();

        currentSegment = ramseteFollower.currentSegment();
        rightSpeed = Units.metersToEncoderTicks(driveSignal.getRight() / 10);
        leftSpeed = Units.metersToEncoderTicks(driveSignal.getLeft() / 10);
        
        

        if (ramseteFollower.getSegmentFraction() >= pathArr[currentIndex].visionPercentage) {
            // For use with vision
            linearVelocity = Units.metersToEncoderTicks(ramseteFollower.linearVelocity / 10);
            
            // alter the drive signal for vision if path says so
            driveSignal = getVisionDriveSignal(linearVelocity);
            rightSpeed = Units.metersToEncoderTicks(driveSignal.getRight());
            leftSpeed = Units.metersToEncoderTicks(driveSignal.getLeft());

            log("currently using vision");
        }


        currentPathFinished = ramseteFollower.isFinished();

        
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
        nextPathIndex = 0;
    }

    /**
     * 
     * @param direction either FORWARD or BACKWARD
     * @param index     the index of the path, 0,1,2
     */
    protected void init(int index) {
        // out of bounds exception..
        if (index < pathArr.length) {
            log("initializing with index: " + index);
            mDrivetrain.configDefaultPIDF();
            driveSignal = new DriveSignal();
            // get the new limelight from the new path
            mVision = pathArr[index].pathLimelight;
            // get the new contour from the new path
            mVision.set(pathArr[index].visionMode);
            // get the new direction from the new path
            currentDirection = pathArr[index].direction;

            currentPathIsForwards = MotionProfileDirection.FORWARD == currentDirection;

            // start odometry if initializing the first path
            if (index == 0) {
                Odometry.getInstance().setInitialOdometry(pathArr[index].getTrajectory());
                Odometry.getInstance().odometryInit();
                log("first path initialized!");
            }


            // new ramsete follower with the path's trajectory
            ramseteFollower = new RamseteFollower(pathArr[index].getTrajectory(), pathArr[index].direction);
            log("contructed new ramseteFollower");
            currentIndex = index;
        } else {
            // log("sequence is finished");
            sequenceFinished = true;
        }
    }

    public void execute() {
        log("CurrentIndex: " + currentIndex);
        log("NextIndex: " + nextPathIndex);
        /**
         * Works by trying to equal current and next pathIndexes by calling void
         * init(int)
         */
        if (currentIndex < nextPathIndex) {
            init(nextPathIndex);
        } 
        if (!sequenceFinished) {
            // log("limelight in use: " + mVision);
            // log("direction: " + currentDirection.asString);
            // uses vision at the percentages specified for the paths during construction
            ramsetePeriodic();
            mDrivetrain.setAutoVelocity(leftSpeed, rightSpeed);
            // log("LeftVel: " + driveSignal.getLeft() + "\nRightVel: " + driveSignal.getRight());

            // if (ramseteFollower.getSegmentFraction() < .5) {
            //     // basically just to make sure robot initialized, should be the same of the
            //     // other path's endElement.aimedState (the path that ended before this one)
            //     stateMachine.setAimedRobotState(pathArr[currentIndex].startElement.aimedState);
            //     // log("calling aimed state from sequence: " + pathArr[currentIndex].startElement.aimedState
            //     //         + "\nHatch grabber is constant");
            //     // don't want to run grab hatch forever lol
            //     mHatchGrabber.runConstant();
            // } else {
            //     stateMachine.setAimedRobotState(pathArr[currentIndex].endElement.aimedState);
            //     // log("calling aimed state from sequence: " + pathArr[currentIndex].endElement.aimedState);
            //     // need to grab hatch from loading station at end of the path if it's going
            //     // backwards
            //     if (!currentPathIsForwards) {
            //         mHatchGrabber.grabHatch();
            //         // log("grabbing hatch");
            //     }
            // }

            if (currentPathFinished) {
                // log("finished path " + currentIndex);
                // need to release hatch at end of the path if it's going forwards
                // if (currentPathIsForwards) {
                //     mHatchGrabber.releaseHatch();
                //     // log("releasing hatch");
                // }
                // If current path ended, go to next path, unless next index doesn't exist, in that case make the next index the last index (2);
                nextPathIndex = nextPathIndex + 1 >= 3 ? 2 : nextPathIndex + 1;
            }
        } else {
            // log("sequence is finished");
            mDrivetrain.stop();
            mDrivetrain.setOpenLoop(0, 0);
            // drivetrainBackwardsNotifier.startSingle(0);
        }

    }

    public String getName() {
        return this.name;
    }

    public String toString() {
        return getName();
    }

    protected DriveSignal getVisionDriveSignal(double linearVelocity) {
        mVision.center();
        log("centering with vision");
        return new DriveSignal(linearVelocity + ( mVision.leftSpeed  * visionVelocityMultiplier ),
                linearVelocity + (mVision.rightSpeed * visionVelocityMultiplier) );
    }

    /**
     * basically prints the argument
     * 
     * @param str to print
     */
    public static void log(String str) {
        System.out.println(str);
    }
}