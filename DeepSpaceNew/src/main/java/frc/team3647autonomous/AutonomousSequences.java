package frc.team3647autonomous;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.team3647subsystems.Arm;
import frc.team3647subsystems.Drivetrain;
import frc.team3647subsystems.Elevator;
import frc.team3647subsystems.HatchGrabber;
import frc.team3647subsystems.SeriesStateMachine;
import frc.team3647subsystems.VisionController;
import frc.team3647subsystems.Arm.ArmPosition;
import frc.team3647subsystems.Elevator.ElevatorLevel;
import frc.team3647subsystems.SeriesStateMachine.ScoringPosition;
import frc.team3647utility.Units;
import jaci.pathfinder.Trajectory;

public class AutonomousSequences 
{
	public static VisionController limelightFourBar = new VisionController("top");
	public static VisionController limelightClimber = new VisionController("bottom");

	public static int autoStep = 0;

	public static DriveSignal driveSignal;
	public static Trajectory trajectory;
	public static RamseteFollower ramseteFollower;
	public static Trajectory.Segment current;
	public static double leftSpeed;
	public static double rightSpeed;
	private static Timer autoTimer;

	public static void autoInit(String trajectoryName) 
	{
		driveSignal = new DriveSignal();
		trajectory = TrajectoryUtil.getTrajectoryFromName(trajectoryName);
		ramseteFollower = new RamseteFollower(trajectory, MotionProfileDirection.FORWARD);
		Odometry.getInstance().setInitialOdometry(trajectory);
		Odometry.getInstance().odometryInit();
		autoTimer = new Timer();
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
			autoStep = 1;
		case 1:
			limelightFourBar.disabledMode();
			ramsetePeriodic();
			Drivetrain.setAutoVelocity(leftSpeed, rightSpeed);
			System.out.println("Ramsete Running: " + !ramseteFollower.isFinished());
			if (ramseteFollower.isFinished())
				autoStep = 2;
			break;

		case 2:
			double threshold = 0.1;
			limelightFourBar.visionTargetingMode();
			limelightFourBar.center(threshold);
			System.out.println("Centering!!");
			Drivetrain.setPercentOutput(limelightFourBar.leftSpeed, limelightFourBar.rightSpeed);
			if (limelightFourBar.centered(threshold)) 
			{
				autoStep = 3;
				SeriesStateMachine.aimedRobotState = ScoringPosition.HATCHL1FORWARDS;
			}
			break;

		case 3:
			Drivetrain.stop();
			Odometry.getInstance().closeNotifier();
			autoTimer.reset();
			autoTimer.start();
			autoStep = 4;
			break;
		case 4:
			System.out.println("Running step 4");
			System.out.println("Auto Timer: " + autoTimer.get());
			limelightFourBar.visionTargetingMode();
			limelightFourBar.center(.1);
			Drivetrain.setPercentOutput(limelightFourBar.leftSpeed, limelightFourBar.rightSpeed);
			if(autoTimer.get() < 1.5)
				Drivetrain.setPercentOutput(.25, .25);
			else if(autoTimer.get() < 3)
			{
				System.out.println("Releasing hatch");
				// HatchGrabber.releaseHatch();
			}
			else if(autoTimer.get() < 5)
				Drivetrain.stop();
			else if(autoTimer.get() < 7)
				Drivetrain.setPercentOutput(-.25, -.25);
			else if(autoTimer.get() < 9)
				Drivetrain.stop();
				autoStep = 6;
			break;
		case 5:
			// if (Arm.currentState == ArmPosition.FLATBACKWARDS && Elevator.currentState == ElevatorLevel.BOTTOM)
				autoStep = 6;
			// else
			// 	SeriesStateMachine.aimedRobotState = ScoringPosition.HATCHL1BACKWARDS;
			break;
		case 6:
			System.out.println("AUTO SEQUENCE FINISHED!");
			Robot.autoNotifier.close();
			break;
		}
	}

}