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
import frc.team3647utility.Units;
import jaci.pathfinder.Trajectory;

public class AutonomousSequences 
{
	public static VisionController limelightFourBar = new VisionController("fourbar");
	public static VisionController limelightClimber = new VisionController("climber");
	// public static VisionController limelightArmTop = new VisionController("top");
	// public static VisionController limelightArmBottom = new VisionController("bottom");

	public static int autoStep = -1;

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
		// limelightClimber.limelight.setToRightContour();
		// afterAutoTimer = new Timer();
		// afterAutoTimer.reset();
		// afterAutoStep = -2;
		// autoStep = -1;
		// hey = false;
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
	static int afterAutoStep = -2;
	static boolean hey = false;
	static boolean initializedRobot = false;

	public static void runPath()
	{
		// initializedRobot = Arm.reachedState(ArmPosition.FLATFORWARDS) && Elevator.reachedState(ElevatorLevel.BOTTOM);

		ramsetePeriodic();
		// System.out.println("leftSpeed = " + leftSpeed + " rightSpeed = " + rightSpeed);
		Drivetrain.setAutoVelocity(leftSpeed, rightSpeed);
		// if((ramseteFollower.isFinished() && initializedRobot) || hey)
		// {
		// 	// System.out.println("After auto");
		// 	hey = true;
		// 	switch(afterAutoStep)
		// 	{
		// 		case -2:
		// 			SeriesStateMachine.aimedRobotState = ScoringPosition.HATCHL2FORWARDS;
		// 			if(Elevator.encoderValue > 10000)
		// 				afterAutoStep = -1;
		// 			break;
		// 		case -1:
		// 			if(limelightClimber.limelight.getValidTarget())
		// 				Drivetrain.setPercentOutput(-.5, .5);
		// 			else
		// 			{
		// 				Drivetrain.stop();
		// 				afterAutoStep = 66;
		// 			}
		// 			break;
		// 		case 66:
		// 			if(limelightClimber.area < 5.5 && limelightClimber.limelight.getValidTarget())
		// 			{
		// 				// System.out.println("CENTERING");
		// 				limelightClimber.center(.0037);
		// 				Drivetrain.setPercentOutput(limelightClimber.leftSpeed + .4, limelightClimber.rightSpeed + .4);
		// 			}
		// 			else
		// 			{
		// 				Drivetrain.stop();
		// 				afterAutoStep = 0;
		// 			}
		// 			break;
		// 		case 0:
		// 			afterAutoTimer.reset();
		// 			afterAutoTimer.start();
		// 			afterAutoStep = 1;
		// 			break;
		// 		case 1:
		// 			Drivetrain.setPercentOutput(.24, .24);
		// 			if(afterAutoTimer.get() > 1)
		// 			{
		// 				Drivetrain.stop();
		// 				afterAutoTimer.reset();
		// 				afterAutoTimer.start();
		// 				afterAutoStep = 2;
		// 			}
		// 			break;
		// 		case 2:
		// 			HatchGrabber.releaseHatch();
		// 			if(afterAutoTimer.get() > .4)
		// 			{
		// 				HatchGrabber.stopMotor();
		// 				afterAutoTimer.reset();
		// 				afterAutoTimer.start();
		// 				afterAutoStep = 3;
		// 			}
		// 			break;
		// 		case 3:
		// 			Drivetrain.setPercentOutput(-.24, -.24);
		// 			if(afterAutoTimer.get() > 1)
		// 			{
		// 				Drivetrain.stop();
		// 				afterAutoTimer.reset();
		// 				afterAutoTimer.start();
		// 				afterAutoStep = 4;
		// 			}
		// 			break;
		// 		case 4:
		// 			SeriesStateMachine.setAimedRobotState(ScoringPosition.HATCHL2BACKWARDS);
		// 			Drivetrain.stop();
		// 			// System.out.println("DONE!");
		// 			autoInitBWD("RocketToStation");
		// 			afterAutoStep = 5;
		// 			break;
		// 		case 5:
		// 			System.out.println("running step 5");
		// 			runPathBWD();
		// 			break;
		// 	}
		// }
	}


	private static int hatchGrabbingCase = -1;
	private static Timer hatchGrabbingTimer = new Timer();
	private static boolean grabbedHatch = false;
	private static boolean hey2 = false;
	public static void runPathBWD()
	{
		System.out.println("running backwards");
		ramsetePeriodic();
		// System.out.println("leftSpeed = " + leftSpeed + " rightSpeed = " + rightSpeed);
		Drivetrain.setAutoVelocity(-rightSpeed, -leftSpeed);
		if(ramseteFollower.isFinished() || hey2)
		{
			System.out.println("AutoStep: " + autoStep + " LIMELIGHT: " + limelightFourBar.limelight.getValidTarget());
			hey2 = true;
			switch(autoStep)
			{
				case -1:
					if(limelightFourBar.limelight.getValidTarget())
					{
						limelightFourBar.limelight.setToRightContour();
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
						SeriesStateMachine.setAimedRobotState(ScoringPosition.HATCHL2BACKWARDS);
						System.out.println("CENTERING");
						limelightFourBar.center(.0037);
						Drivetrain.setPercentOutput(-limelightFourBar.rightSpeed - .4, -limelightFourBar.leftSpeed - .4);					
					}
					else
					{
						if(Elevator.encoderValue != 0 || !Arm.reachedState(ArmPosition.FLATBACKWARDS))
							SeriesStateMachine.setAimedRobotState(ScoringPosition.HATCHL1BACKWARDS);
						else
							autoStep = 0;
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
					if(Robot.pDistributionPanel.getCurrent(6) > 17 && afterAutoTimer.get() > .4)
					{
						hatchGrabbingCase = 0;
					}
					else if(afterAutoTimer.get() > 1)
					{
						Drivetrain.setPercentOutput(.2, .2);
						afterAutoStep = 66;
						hatchGrabbingCase = -1;
					}
					switch(hatchGrabbingCase)
					{
						case 0:
							hatchGrabbingTimer.reset();
							hatchGrabbingTimer.start();
							hatchGrabbingCase = 1;
							break;
						case 1:
							Drivetrain.setPercentOutput(.2, .2);
							if(hatchGrabbingTimer.get() > 1.3)
							{
								Drivetrain.stop();
								if(Robot.pDistributionPanel.getCurrent(6) > 17)
								{
									grabbedHatch = true;
									hatchGrabbingCase = -1;
								}
								else
								{
									grabbedHatch = false;
									autoStep = 66;
									hatchGrabbingCase = -1;
								}	
							}
							break;
						case 2:
							Drivetrain.setPercentOutput(.18, .18);
							afterAutoStep = 66;
							break;
					}
					if(grabbedHatch)
					{
						HatchGrabber.runConstant();
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
						SeriesStateMachine.setAimedRobotState(ScoringPosition.HATCHL2FORWARDS);
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
