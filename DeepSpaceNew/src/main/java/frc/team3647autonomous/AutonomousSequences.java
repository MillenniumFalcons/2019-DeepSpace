package frc.team3647autonomous;

import edu.wpi.first.wpilibj.Timer;
import frc.team3647subsystems.Drivetrain;
import frc.team3647subsystems.HatchGrabber;
import frc.team3647subsystems.VisionController;
import frc.team3647utility.Units;
import jaci.pathfinder.Trajectory;

public class AutonomousSequences
{
    public static VisionController limelightTop = new VisionController("top");
    public static VisionController limelightBottom = new VisionController("bottom");

    public static int autoStep = 0;

    public static DriveSignal driveSignal;
    public static Trajectory trajectory;
    public static RamseteFollower ramseteFollower;
    public static Trajectory.Segment current;
    public static double leftSpeed;
    public static double rightSpeed;

    public static void autoInit(String trajectoryName)
    {
        driveSignal = new DriveSignal();
        trajectory = TrajectoryUtil.getTrajectoryFromName(trajectoryName);
        ramseteFollower = new RamseteFollower(trajectory, MotionProfileDirection.FORWARD);
        Odometry.getInstance().setInitialOdometry(trajectory);
        Odometry.getInstance().odometryInit();
    }

    public static void ramsetePeriodic()
    {
        driveSignal = ramseteFollower.getNextDriveSignal();
        current = ramseteFollower.currentSegment();
        rightSpeed = Units.metersToEncoderTicks(driveSignal.getRight() / 10);
        leftSpeed = Units.metersToEncoderTicks(driveSignal.getLeft() / 10);
    }

    public static void level2RightToCargoShipRight()
    {

        switch (autoStep) 
        {
        case 0:
            limelightTop.disabledMode();
            ramsetePeriodic();
            Drivetrain.setAutoVelocity(leftSpeed, rightSpeed);
            System.out.println("Ramsete Running: " + !ramseteFollower.isFinished());
            if (ramseteFollower.isFinished())
                autoStep = 1;
            break;

        case 1:
            limelightTop.visionTargetingMode();
            limelightTop.center(1, .035, 0.15, 0.1);
            System.out.println("Centering!!");
            Drivetrain.setPercentOutput(limelightTop.leftSpeed, limelightTop.rightSpeed);
            if(limelightTop.leftSpeed == 0 && limelightTop.rightSpeed == 0)
                autoStep = 2;
            break;
        
        case 2:
            Drivetrain.stop();
            System.out.println("AUTO SEQUENCE FINISHED!");
            autoStep = 3;
            break;
        case 3:
            Drivetrain.setPercentOutput(.35, .35);
            Timer.delay(.5);
            HatchGrabber.releaseHatch();
            Drivetrain.setPercentOutput(-.25, -.25);
            Timer.delay(.5);
            Drivetrain.stop();
            autoStep = 4;
            break;
        case 4:
            
            break;
        }
    }

}