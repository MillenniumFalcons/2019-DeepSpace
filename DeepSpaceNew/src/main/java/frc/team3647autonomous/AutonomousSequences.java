package frc.team3647autonomous;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.team3647subsystems.Arm;
import frc.team3647subsystems.Drivetrain;
import frc.team3647subsystems.Elevator;
import frc.team3647subsystems.HatchGrabber;
import frc.team3647subsystems.SeriesStateMachine;
import frc.team3647subsystems.VisionController;
import frc.team3647subsystems.Arm.ArmPosition;
// import frc.team3647subsystems.Elevator.ElevatorLevel;
import frc.team3647subsystems.SeriesStateMachine.ScoringPosition;
import frc.team3647subsystems.VisionController.VisionMode;
import frc.team3647utility.Units;
import jaci.pathfinder.Trajectory;

public class AutonomousSequences 
{
	public static VisionController limelightFourBar = new VisionController("fourbar");
	public static VisionController limelightClimber = new VisionController("climber");
	
	public static int autoStep = 0;
	private static Timer autoTimer = new Timer(); 


	public static DriveSignal driveSignal;
	public static Trajectory trajectory;
	public static RamseteFollower ramseteFollower;
	public static Trajectory.Segment current;
	public static double leftSpeed;
	public static double rightSpeed;

	public static void autoInitFWD(String trajectoryName) 
	{
		Drivetrain.selectPIDF(Constants.velocitySlotIdx, Constants.rightVelocityPIDF, Constants.leftVelocityPIDF);
		driveSignal = new DriveSignal();
		trajectory = TrajectoryUtil.getTrajectoryFromName(trajectoryName);
		ramseteFollower = new RamseteFollower(trajectory, MotionProfileDirection.FORWARD);
		Odometry.getInstance().setInitialOdometry(trajectory);
		Odometry.getInstance().odometryInit();
		autoStep = 0;
		autoTimer.stop();
		autoTimer.reset();
		limelightClimber.set(VisionMode.kClosest);
	}

	public static void autoInitBWD(String trajectoryName) 
	{
		Drivetrain.selectPIDF(Constants.velocitySlotIdx, Constants.rightVelocityPIDF, Constants.leftVelocityPIDF);
		driveSignal = new DriveSignal();
		trajectory = TrajectoryUtil.getTrajectoryFromName(trajectoryName);
		ramseteFollower = new RamseteFollower(trajectory, MotionProfileDirection.BACKWARD);
	}

	public static void testing()
	{
		driveSignal = ramseteFollower.getNextDriveSignal();
    	current = ramseteFollower.currentSegment();

    	rightSpeed = Units.metersToEncoderTicks(driveSignal.getRight() / 10);
    	leftSpeed = Units.metersToEncoderTicks(driveSignal.getLeft() / 10);

		if(ramseteFollower.isFinished())
		{
			ramseteFollower = new RamseteFollower(trajectory, MotionProfileDirection.BACKWARD);
			Odometry.getInstance().setInitialOdometry(TrajectoryUtil.reversePath(trajectory));
		}
	}

	public static void ramsetePeriodic() 
	{
		driveSignal = ramseteFollower.getNextDriveSignal();
		current = ramseteFollower.currentSegment();
		rightSpeed = Units.metersToEncoderTicks(driveSignal.getRight() / 10);
		leftSpeed = Units.metersToEncoderTicks(driveSignal.getLeft() / 10);
	}

	public static void runPath()
	{
		ramsetePeriodic();
		Drivetrain.setAutoVelocity(leftSpeed, rightSpeed);

		if(ramseteFollower.isFinished())
		{
			switch(autoStep)
			{
				case 0:
					if(limelightClimber.limelight.getValidTarget() && limelightClimber.area < Constants.limelightAreaThreshold)
					{
						limelightClimber.center();
						Drivetrain.setPercentOutput(limelightClimber.leftSpeed + .35, limelightClimber.rightSpeed + .35);
					}
					else if(!limelightClimber.limelight.getValidTarget())
					{
						Drivetrain.setPercentOutput(.55, + -.75);;
					}
					else
					{
						Drivetrain.stop();
						autoStep = 1;
						autoTimer.reset();
						autoTimer.start();
					}
					break;
				case 1:
					if(autoTimer.get() < 0.7)
					{
						Drivetrain.setAutoVelocity(500, 500);
					}
					else
					{
						autoTimer.stop();
						autoTimer.reset();
						autoStep = 2;
						autoTimer.start();
					}
					break;
				case 2:
					if(autoTimer.get() < 0.5)
					{
						HatchGrabber.releaseHatch();
					}
					else
					{
						HatchGrabber.stopMotor();
						autoStep = 3;
						autoTimer.stop();
						autoTimer.reset();
						autoTimer.start();
					}
					break;
				case 3:
					if(autoTimer.get() < 1)
					{
						Drivetrain.setAutoVelocity(-500, -500);
					}
					else
					{
						Drivetrain.stop();
						autoStep = 4;
						autoTimer.stop();
						autoTimer.reset();
					}
					break;
				case 4:
					System.out.println("1 Hatch Auto DONE!");
					break;
			}
		}
	}
}
