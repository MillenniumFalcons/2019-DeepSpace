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
import frc.team3647subsystems.Elevator.ElevatorLevel;
import frc.team3647subsystems.SeriesStateMachine.ScoringPosition;
import frc.team3647utility.Units;
import jaci.pathfinder.Trajectory;

public class AutonomousSequences 
{
	public static VisionController limelightFourBar = new VisionController("fourbar");
	public static VisionController limelightClimber = new VisionController("climber");
	public static VisionController limelightArmTop = new VisionController("top");
	public static VisionController limelightArmBottom = new VisionController("bottom");

	public static int autoStep = -1;

	public static DriveSignal driveSignal;
	public static Trajectory trajectory;
	public static RamseteFollower ramseteFollower;
	public static Trajectory.Segment current;
	public static double leftSpeed;
	public static double rightSpeed;
	private static Timer autoTimer;

	public static void autoInitFWD(String trajectoryName) 
	{
		Drivetrain.selectPIDF(Constants.velocitySlotIdx, Constants.rightVelocityPIDF, Constants.leftVelocityPIDF);
		driveSignal = new DriveSignal();
		trajectory = TrajectoryUtil.getTrajectoryFromName(trajectoryName);
		ramseteFollower = new RamseteFollower(trajectory, MotionProfileDirection.FORWARD);
		Odometry.getInstance().setInitialOdometry(trajectory);
		Odometry.getInstance().odometryInit();
		limelightClimber.limelight.setToRightContour();
		afterAutoTimer = new Timer();
		afterAutoTimer.reset();
		afterAutoStep = -1;
		hey = false;
	}

	public static void autoInitBWD(String trajectoryName) 
	{
		Drivetrain.selectPIDF(Constants.velocitySlotIdx, Constants.rightVelocityPIDF, Constants.leftVelocityPIDF);
		driveSignal = new DriveSignal();
		trajectory = TrajectoryUtil.getTrajectoryFromName(trajectoryName);
		ramseteFollower = new RamseteFollower(trajectory, MotionProfileDirection.BACKWARD);
		hey = false;
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

	static Timer afterAutoTimer = new Timer();
	static int afterAutoStep = -1;
	static boolean hey = false;

	public static void runPath()
	{
		ramsetePeriodic();
		// System.out.println("leftSpeed = " + leftSpeed + " rightSpeed = " + rightSpeed);
		Drivetrain.setAutoVelocity(leftSpeed, rightSpeed);
		if(ramseteFollower.isFinished() || hey)
		{
			hey = true;
			switch(afterAutoStep)
			{
				case -1:
					if(limelightClimber.limelight.getValidTarget())
						Drivetrain.setPercentOutput(-.5, .5);
					else
					{
						Drivetrain.stop();
						afterAutoStep = 66;
					}
					break;
				case 66:
					if(limelightClimber.area < 5.5 && limelightClimber.limelight.getValidTarget())
					{
						// System.out.println("CENTERING");
						limelightClimber.center(.0037);
						Drivetrain.setPercentOutput(limelightClimber.leftSpeed + .4, limelightClimber.rightSpeed + .4);
					}
					else
					{
						Drivetrain.stop();
						afterAutoStep = 0;
					}
					break;
				case 0:
					afterAutoTimer.reset();
					afterAutoTimer.start();
					afterAutoStep = 1;
					break;
				case 1:
					Drivetrain.setPercentOutput(.24, .24);
					if(afterAutoTimer.get() > 1)
					{
						Drivetrain.stop();
						afterAutoTimer.reset();
						afterAutoTimer.start();
						afterAutoStep = 2;
					}
					break;
				case 2:
					HatchGrabber.releaseHatch();
					if(afterAutoTimer.get() > .4)
					{
						HatchGrabber.stopMotor();
						afterAutoTimer.reset();
						afterAutoTimer.start();
						afterAutoStep = 4;
					}
					break;
				case 3:
					Drivetrain.setPercentOutput(-.24, -.24);
					if(afterAutoTimer.get() > 1)
					{
						Drivetrain.stop();
						afterAutoTimer.reset();
						afterAutoTimer.start();
						afterAutoStep = 4;
					}
					break;
				case 4:
					Drivetrain.stop();
					// System.out.println("DONE!");
					autoInitBWD("RocketToStation");
					limelightFourBar.limelight.setToRightContour();
					afterAutoStep = 5;
					break;
				case 5:
					runPathBWD();
					break;
			}
		}
	}

	public static void runPathBWD()
	{
		ramsetePeriodic();
		// System.out.println("leftSpeed = " + leftSpeed + " rightSpeed = " + rightSpeed);
		Drivetrain.setAutoVelocity(-rightSpeed, -leftSpeed);
		if(ramseteFollower.isFinished() || hey)
		{
			// System.out.println("AutoStep: " + autoStep + " LIMELIGHT: " + limelightFourBar.limelight.getValidTarget());
			hey = true;
			switch(autoStep)
			{
				case -1:
					if(limelightFourBar.limelight.getValidTarget())
					{
						Drivetrain.setPercentOutput(.65, -.65);
					}
					else
					{
						Drivetrain.stop();
						autoStep = 66;
						// System.out.println("AUTO STEP: " + autoStep);
					}
					break;
				case 66:
					if(limelightFourBar.area < 5)
					{
						
						// System.out.println("CENTERING");
						limelightFourBar.center(.0037);
						Drivetrain.setPercentOutput(-limelightFourBar.rightSpeed - .4, -limelightFourBar.leftSpeed - .4);					
					}
					else
					{
						// Timer.delay(.1);
						// if(limelightFourBar.limelight.getValidTarget() > 1)
						// {
						// 	autoStep = 66;
						// }
						// else
						// {
						Drivetrain.stop();
						autoStep = 0;
						// }
					}
					break;
				case 0:
					afterAutoTimer.reset();
					afterAutoTimer.start();
					autoStep = 1;
					break;
				case 1:
					Drivetrain.setPercentOutput(-.24, -.24);
					if(afterAutoTimer.get() > 1)
					{
						Drivetrain.stop();
						afterAutoTimer.reset();
						afterAutoTimer.start();
						autoStep = 2;
					}
					break;
				case 2:
					HatchGrabber.grabHatch();
					if(afterAutoTimer.get() > .4)
					{
						HatchGrabber.stopMotor();
						afterAutoTimer.reset();
						afterAutoTimer.start();
						autoStep = 3;
					}
					break;
				case 3:
					Drivetrain.setPercentOutput(.24, .24);
					if(afterAutoTimer.get() > 1)
					{
						Drivetrain.stop();
						afterAutoTimer.reset();
						afterAutoTimer.start();
						autoStep = 4;
					}
					break;
				case 4:
					Drivetrain.stop();
					// System.out.println("DONE!");
					break;
			}
		}
	}

}
