package frc.team3647autonomous;

import edu.wpi.first.wpilibj.Notifier;
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
		autoTimer.stop();
		autoTimer.reset();
		limelightClimber.set(VisionMode.kClosest);
	}

	public static void autoInitFWD2(String trajectoryName) 
	{
		Drivetrain.selectPIDF(Constants.velocitySlotIdx, Constants.rightVelocityPIDF, Constants.leftVelocityPIDF);
		driveSignal = new DriveSignal();
		trajectory = TrajectoryUtil.getTrajectoryFromName(trajectoryName);
		ramseteFollower = new RamseteFollower(trajectory, MotionProfileDirection.FORWARD);
		limelightClimber.set(VisionMode.kClosest);
	}

	public static void autoInitBWD(String trajectoryName) 
	{
		Drivetrain.selectPIDF(Constants.velocitySlotIdx, Constants.rightVelocityPIDF, Constants.leftVelocityPIDF);
		driveSignal = new DriveSignal();
		trajectory = TrajectoryUtil.getTrajectoryFromName(trajectoryName);
		ramseteFollower = new RamseteFollower(trajectory, MotionProfileDirection.BACKWARD);
		limelightFourBar.set(VisionMode.kClosest);
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
		Drivetrain.setAutoVelocity(leftSpeed*.97, rightSpeed*.97);

		if(ramseteFollower.isFinished())
		{
			switch(autoStep)
			{
				case 0:
					if(limelightClimber.limelight.getValidTarget() && limelightClimber.area < Constants.limelightAreaThreshold)
					{
						limelightClimber.center();
						Drivetrain.setPercentOutput(.75*limelightClimber.leftSpeed + .4, .75*limelightClimber.rightSpeed + .4);
					}
					else if(!limelightClimber.limelight.getValidTarget())
					{
						Drivetrain.setPercentOutput(-.55, +.55);;
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
					if(autoTimer.get() < 0.4)
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
					if(autoTimer.get() < 0.2)
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
					autoInitBWD("LeftRocketToStation");
					limelightClimber.set(VisionMode.kBlack);
					limelightFourBar.set(VisionMode.kClosest);
					SeriesStateMachine.aimedRobotState = ScoringPosition.HATCHL1BACKWARDS;
					System.out.println("1 Hatch Auto DONE!");
					autoStep = 5;
					break;
				case 5:
					if(limelightFourBar.limelight.getValidTarget() && limelightFourBar.area < Constants.limelightAreaThreshold)
					{
						limelightFourBar.center();
						Drivetrain.setPercentOutput(.75*limelightFourBar.leftSpeed - .55, .75*limelightFourBar.rightSpeed - .55);
					}
					else if(!limelightFourBar.limelight.getValidTarget())
					{
						Drivetrain.setPercentOutput(.25, -.35);;
					}
					else
					{
						Drivetrain.stop();
						autoStep = 6;
						autoTimer.reset();
						autoTimer.start();
					}
					break;
				case 6:
					if(autoTimer.get() < 1)
					{
						HatchGrabber.grabHatch();
						Drivetrain.setAutoVelocity(-500, -500);
					}
					else
					{
						autoTimer.stop();
						autoTimer.reset();
						autoStep = 7;
						autoTimer.start();
					}
					break;
				case 7:
					if(autoTimer.get() < 0.8)
					{
						HatchGrabber.grabHatch();
					}
					else
					{
						HatchGrabber.stopMotor();
						autoStep = 8;
						autoTimer.stop();
						autoTimer.reset();
						// autoTimer.start();
					}
					break;
				case 8:
					autoStep = 9;
					break;
				case 9:
					System.out.println("Grabbed Second Hatch!");
					autoInitFWD2("StationToLeftRocket");
					limelightFourBar.set(VisionMode.kBlack);
					limelightClimber.set(VisionMode.kClosest);
					autoStep = 10;
					break;
				case 10:
					SeriesStateMachine.aimedRobotState = ScoringPosition.HATCHL2FORWARDS;
					System.out.println("Worked?");
					autoStep = 11;
					break;
				case 11:
					if(limelightClimber.limelight.getValidTarget() && limelightClimber.area < Constants.limelightAreaThreshold)
					{
						limelightClimber.center();
						Drivetrain.setPercentOutput(limelightClimber.leftSpeed + .55, limelightClimber.rightSpeed + .55);
					}
					else if(!limelightClimber.limelight.getValidTarget())
					{
						Drivetrain.setPercentOutput(.55, + -.75);;
					}
					else
					{
						Drivetrain.stop();
						autoStep = 12;
						autoTimer.reset();
						autoTimer.start();
					}
					break;
				case 12:
					if(autoTimer.get() < 0.7)
					{
						Drivetrain.setAutoVelocity(500, 500);
					}
					else
					{
						autoTimer.stop();
						autoTimer.reset();
						autoStep = 13;
						autoTimer.start();
					}
					break;
				case 13:
					if(autoTimer.get() < 0.5)
					{
						HatchGrabber.releaseHatch();
					}
					else
					{
						HatchGrabber.stopMotor();
						autoStep = 14;
						autoTimer.stop();
						autoTimer.reset();
						autoTimer.start();
					}
					break;
				case 14:
					System.out.println("2 HATCH AUTOO!!!!!");
					if(autoTimer.get() < 0.5)
					{
						Drivetrain.setAutoVelocity(-500, -500);
					}
					else
					{
						Drivetrain.stop();
						autoTimer.stop();
						autoTimer.reset();
					}
					break;
			}
		}
	}

	public static void runPath2()
	{
		if(true)
		{
			switch(autoStep)
			{
				case 0:
					if(limelightClimber.limelight.getValidTarget() && limelightClimber.area < Constants.limelightAreaThreshold)
					{
						limelightClimber.center();
						Drivetrain.setPercentOutput(limelightClimber.leftSpeed + .25, limelightClimber.rightSpeed + .25);
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
