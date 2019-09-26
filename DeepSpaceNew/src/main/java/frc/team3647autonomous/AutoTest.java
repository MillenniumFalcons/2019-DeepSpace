package frc.team3647autonomous;

import edu.wpi.first.wpilibj.Notifier;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.team3647utility.Units;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;

public class AutoTest {
    public static RamseteFollower ramseteFollower;
    public static DriveSignal driveSignal;
    public static Trajectory trajectory;
    public static Segment segment;
    public static double leftSpeed, rightSpeed;
    public static String xtrajectoryName;
    public static MotionProfileDirection direction =  MotionProfileDirection.BACKWARD;

    public static Notifier odometryNotifier = new Notifier(() -> {
        Robot.mDrivetrain.updateEncoders();
        Robot.gyro.update();
        Odometry.getInstance().runOdometry(Robot.mDrivetrain);
    });

    public static Notifier pathNotifier = new Notifier(() -> {
        ramsetePeriodic();
        Log("RightWanted: " + rightSpeed + "\nLeftWanted: " + leftSpeed);

        Robot.mDrivetrain.setAutoVelocity(leftSpeed, rightSpeed);
        Log("RightActual: " + Robot.mDrivetrain.getRightVelocity() + "\nLeftActual: " + Robot.mDrivetrain.getLeftVelocity());
        Log("Desired heading: " + segment.heading + "\nactual heading: " + Math.toRadians(Robot.gyro.getYaw()));

        if(ramseteFollower.isFinished() && direction==MotionProfileDirection.FORWARD) {
        }
    });

    public static void init(String trajectoryName) {
        xtrajectoryName = trajectoryName;
        Robot.mDrivetrain.selectPIDF(Constants.velocitySlotIdx, Constants.rightVelocityPIDF,
                Constants.leftVelocityPIDF);
        driveSignal = new DriveSignal();
        trajectory = TrajectoryUtil.getTrajectoryFromName(trajectoryName);
        ramseteFollower = new RamseteFollower(trajectory, MotionProfileDirection.FORWARD);
        Odometry.getInstance().setInitialOdometry(trajectory);
        Odometry.getInstance().odometryInit();
        odometryNotifier.startPeriodic(.01);
        pathNotifier.startPeriodic(.02);
    }

    public static void initBwd() {
        driveSignal = new DriveSignal();
        trajectory = TrajectoryUtil.reversePath(TrajectoryUtil.getTrajectoryFromName(xtrajectoryName));
        ramseteFollower = new RamseteFollower(trajectory, MotionProfileDirection.BACKWARD);
        direction = MotionProfileDirection.BACKWARD;
    }

    public static void ramsetePeriodic() {
        driveSignal = ramseteFollower.getNextDriveSignal();
        segment = ramseteFollower.currentSegment();
        rightSpeed = Units.metersToEncoderTicks(driveSignal.getRight() / 10);
        leftSpeed = Units.metersToEncoderTicks(driveSignal.getLeft() / 10);
    }

    public static void Log(String str) {
        System.out.print(str);
    }


    public static void finish() {
        pathNotifier.stop();
        odometryNotifier.stop();
        Robot.mDrivetrain.setOpenLoop(0, 0);
        Robot.mDrivetrain.stop();        
    }
}