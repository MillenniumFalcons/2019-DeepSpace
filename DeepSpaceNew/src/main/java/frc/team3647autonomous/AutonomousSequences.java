package frc.team3647autonomous;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.team3647subsystems.Drivetrain;
import frc.team3647subsystems.HatchGrabber;
import frc.team3647subsystems.SeriesStateMachine;
import frc.team3647subsystems.VisionController;
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
	public static double leftSpeedLin;
	public static double rightSpeedLin;
	private static boolean grabbedSecondHatch = false;

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
		grabbedSecondHatch = false;
		autoStep = 0;
		
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
		rightSpeedLin = Units.metersToEncoderTicks(ramseteFollower.linearVelocity / 10);
		leftSpeedLin = Units.metersToEncoderTicks(ramseteFollower.linearVelocity / 10);
	}

	public static void runVision()
	{
		ramsetePeriodic();
		if(!ramseteFollower.pathFractionSegment(.5) && !ramseteFollower.isFinished())
		{
			Drivetrain.setAutoVelocity(leftSpeed, rightSpeed);
		}
		else if(ramseteFollower.pathFractionSegment(.5))
		{
			limelightClimber.center();
			Drivetrain.setAutoVelocity(leftSpeed + (2500)*limelightClimber.leftSpeed, rightSpeed + (2500)*limelightClimber.rightSpeed);
			// System.out.println(limelightClimber.leftSpeed);
		}
		else if(ramseteFollower.isFinished() && limelightClimber.y > Constants.limelightYOffset)
		{
			limelightClimber.center();
			Drivetrain.setAutoVelocity((2500)*limelightClimber.leftSpeed, (2500)*limelightClimber.rightSpeed);
		}
		else
		{
			HatchGrabber.releaseHatch();
			Drivetrain.setAutoVelocity(-500, -500);
		}
	}

	public static void runPath()
	{
		ramsetePeriodic();
		Drivetrain.setAutoVelocity(leftSpeed, rightSpeed);
		// System.out.println(ramseteFollower.pathHalfway());
	}

	public static void rocketAutoIntegrated()
	{
		ramsetePeriodic();
		switch(autoStep)
		{
			case 0:
				if(!ramseteFollower.pathFractionSegment(.6) && !ramseteFollower.isFinished())
				{
					System.out.println("running first path");
					Drivetrain.setAutoVelocity(leftSpeed, rightSpeed);
				}
				else
				{
					autoStep = 1;
				}
				break;
			case 1:
				if(ramseteFollower.pathFractionSegment(.6))
				{
					SeriesStateMachine.aimedRobotState = ScoringPosition.HATCHL1FORWARDS;
					limelightClimber.center();
					Drivetrain.setAutoVelocity(leftSpeedLin + (2500)*limelightClimber.leftSpeed, rightSpeedLin + (2500)*limelightClimber.rightSpeed);
				}
				else if(ramseteFollower.isFinished())
				{
					System.out.println("First path finished");
					HatchGrabber.releaseHatch();
					autoStep = 4;
				}
				break;
			case 4:
				autoInitBWD("LeftRocketToStation");
				System.out.println("second path started");
				limelightClimber.set(VisionMode.kBlack);
				limelightFourBar.set(VisionMode.kRight);
				autoStep = 5;
				break;
			case 5:
				if(!ramseteFollower.pathFractionSegment(.5) && !ramseteFollower.isFinished())
				{
					Drivetrain.setAutoVelocity(leftSpeed, rightSpeed);
					if(ramseteFollower.pathFractionSegment(.15))
					{
						SeriesStateMachine.aimedRobotState = ScoringPosition.HATCHL1BACKWARDS;
						HatchGrabber.stopMotor();
					}
				}
				else
				{
					autoStep = 6;
				}
				break;
			case 6:
				if(ramseteFollower.pathFractionSegment(.5))
				{
					System.out.println("Going to loading statio");
					limelightFourBar.center();
					HatchGrabber.grabHatch();
					Drivetrain.setAutoVelocity(leftSpeedLin +  (2500)*limelightFourBar.leftSpeed,  rightSpeedLin + (2500)*limelightFourBar.rightSpeed);
				}
				else
				{
					grabbedSecondHatch = true;
					autoInitFWD2("StationToLeftRocket");
					limelightFourBar.set(VisionMode.kBlack);
					limelightClimber.set(VisionMode.kLeft);
					autoStep = 10;
				}
				break;
			case 10:
				if(!ramseteFollower.pathFractionSegment(.5) && !ramseteFollower.isFinished())
				{
					Drivetrain.setAutoVelocity(leftSpeed, rightSpeed);
					if(ramseteFollower.pathFractionSegment(.2))
					{
						SeriesStateMachine.aimedRobotState = ScoringPosition.HATCHL2FORWARDS;
						HatchGrabber.runConstant();
					}	
				}
				else
				{
					autoStep = 11;
				}
				break;
			case 11:
				if(ramseteFollower.pathFractionSegment(.6))
				{
					limelightClimber.center();
					Drivetrain.setAutoVelocity(leftSpeedLin + (2500)*limelightClimber.leftSpeed, rightSpeedLin + (2500)*limelightClimber.rightSpeed);
				}
				else if(ramseteFollower.isFinished())
				{
					HatchGrabber.releaseHatch();
					autoStep = 13;
				}
				break;
			case 13:
				if(autoTimer.get() < 0.3)
				{
					HatchGrabber.releaseHatch();
					Drivetrain.setAutoVelocity(-500, -500);
				}
				else
				{
					HatchGrabber.stopMotor();
					//Drivetrain.stop();
					autoStep = 14;
					autoTimer.stop();
					autoTimer.reset();
				}
				break;
			case 14:
				//TwoHatchAuto.exe failed
				break;
			default:
				Drivetrain.stop();
				break;
		}
	}

	public static void sideCargoShipAuto()
	{
		ramsetePeriodic();
		switch(autoStep)
		{
			case 0:
				if(!ramseteFollower.pathFractionSegment(.8) && !ramseteFollower.isFinished())
				{
					System.out.println("running first path");
					Drivetrain.setAutoVelocity(leftSpeed, rightSpeed);
				}
				else
				{
					limelightClimber.set(VisionMode.kRight);
					autoStep = 1;
				}
				break;
			case 1:
				if(ramseteFollower.pathFractionSegment(.8))
				{
					SeriesStateMachine.aimedRobotState = ScoringPosition.HATCHL1FORWARDS;
					limelightClimber.center();
					Drivetrain.setAutoVelocity(leftSpeedLin + (2500)*limelightClimber.leftSpeed, rightSpeedLin + (2500)*limelightClimber.rightSpeed);
				}
				else if(ramseteFollower.isFinished())
				{
					System.out.println("First path finished");
					HatchGrabber.releaseHatch();
					autoStep = 4;
				}
				break;
			case 4:
				autoInitBWD("LeftCargoShipBay1ToStation");
				System.out.println("second path started");
				limelightClimber.set(VisionMode.kBlack);
				limelightFourBar.set(VisionMode.kRight);
				autoStep = 5;
				break;
			case 5:
				if(!ramseteFollower.pathFractionSegment(.6) && !ramseteFollower.isFinished())
				{
					Drivetrain.setAutoVelocity(leftSpeed, rightSpeed);
					if(ramseteFollower.pathFractionSegment(.15))
					{
						SeriesStateMachine.aimedRobotState = ScoringPosition.HATCHL1BACKWARDS;
						HatchGrabber.stopMotor();
					}
				}
				else
				{
					autoStep = 6;
				}
				break;
			case 6:
				if(ramseteFollower.pathFractionSegment(.6))
				{
					System.out.println("Going to loading statio");
					limelightFourBar.center();
					HatchGrabber.grabHatch();
					Drivetrain.setAutoVelocity(leftSpeedLin +  (2500)*limelightFourBar.leftSpeed,  rightSpeedLin + (2500)*limelightFourBar.rightSpeed);
				}
				else
				{
					grabbedSecondHatch = true;
					System.out.println("Transition to next path");
					autoInitFWD2("StationToLeftCargoShipBay2");
					limelightFourBar.set(VisionMode.kBlack);
					limelightClimber.set(VisionMode.kClosest);
					autoStep = 10;
				}
				break;
			case 10:
				if(!ramseteFollower.pathFractionSegment(.8) && !ramseteFollower.isFinished())
				{
					System.out.println("Third path");
					Drivetrain.setAutoVelocity(leftSpeed, rightSpeed);
					if(ramseteFollower.pathFractionSegment(.2))
					{
						SeriesStateMachine.aimedRobotState = ScoringPosition.HATCHL1FORWARDS;
						HatchGrabber.runConstant();
					}	
				}
				else
				{
					autoStep = 11;
				}
				break;
			case 11:
				if(ramseteFollower.pathFractionSegment(.8))
				{
					limelightClimber.center();
					Drivetrain.setAutoVelocity(leftSpeedLin + (2500)*limelightClimber.leftSpeed, rightSpeedLin + (2500)*limelightClimber.rightSpeed);
				}
				else if(ramseteFollower.isFinished())
				{
					HatchGrabber.releaseHatch();
					autoStep = 13;
				}
				break;
			case 13:
				if(autoTimer.get() < 0.3)
				{
					HatchGrabber.releaseHatch();
					Drivetrain.setAutoVelocity(-500, -500);
				}
				else
				{
					HatchGrabber.stopMotor();
					//Drivetrain.stop();
					autoStep = 14;
					autoTimer.stop();
					autoTimer.reset();
				}
				break;
			case 14:
				//TwoHatchAuto.exe failed
				break;
			default:
				Drivetrain.stop();
				break;
		}
	}


	public static void rocketAuto()
	{
		ramsetePeriodic();
		Drivetrain.setAutoVelocity(leftSpeed*.97, rightSpeed*.97);

		if(grabbedSecondHatch)
			SeriesStateMachine.aimedRobotState = ScoringPosition.HATCHL2FORWARDS;

		if(ramseteFollower.isFinished())
		{
			switch(autoStep)
			{
				case 0:
					if(limelightClimber.limelight.getValidTarget() && limelightClimber.y < Constants.limelightYOffset)
					{
						limelightClimber.center();
						Drivetrain.setPercentOutput(limelightClimber.leftSpeed + .65, limelightClimber.rightSpeed + .65);
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
						Drivetrain.setAutoVelocity(600, 600);
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
						Drivetrain.setAutoVelocity(-600, -600);
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
					// System.out.println("1 Hatch Auto DONE!");
					autoStep = 5;
					break;
				case 5:
					if(limelightFourBar.limelight.getValidTarget() && limelightFourBar.y < Constants.limelightYOffset)
					{
						limelightFourBar.center();
						Drivetrain.setPercentOutput(limelightFourBar.leftSpeed - .55, limelightFourBar.rightSpeed - .55);
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
					if(autoTimer.get() < .7)
					{
						HatchGrabber.grabHatch();
						Drivetrain.setAutoVelocity(-600, -600);
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
					if(autoTimer.get() < 0.25)
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
					// System.out.println("Grabbed Second Hatch!");
					grabbedSecondHatch = true;
					autoInitFWD2("StationToLeftRocket");
					limelightFourBar.set(VisionMode.kBlack);
					limelightClimber.set(VisionMode.kClosest);
					autoStep = 10;
					break;
				case 10:
					// System.out.println("Worked?");
					autoStep = 11;
					break;
				case 11:
					if(limelightClimber.limelight.getValidTarget() && limelightClimber.y < Constants.limelightYOffset)
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
					if(autoTimer.get() < 0.35567)
					{
						Drivetrain.setAutoVelocity(600, 600);
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
					if(autoTimer.get() < 0.55)
					{
						HatchGrabber.releaseHatch();
					}
					else
					{
						HatchGrabber.releaseHatch();
						autoStep = 14;
						autoTimer.stop();
						autoTimer.reset();
						autoTimer.start();
					}
					break;
				case 14:
					// System.out.println("2 HATCH AUTOO!!!!!");
					if(autoTimer.get() < 0.1)
					{
						HatchGrabber.releaseHatch();
						Drivetrain.setAutoVelocity(-600, -600);
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

	public static void cargoShipAuto() //Cargoship Auto
	{
		ramsetePeriodic();
		Drivetrain.setAutoVelocity(leftSpeed, rightSpeed);

		if(grabbedSecondHatch)
			SeriesStateMachine.aimedRobotState = ScoringPosition.HATCHL2FORWARDS;

		if(ramseteFollower.isFinished())
		{
			switch(autoStep)
			{
				case 0:
					if(limelightClimber.limelight.getValidTarget() && limelightClimber.y < Constants.limelightYOffset)
					{
						limelightClimber.center();
						Drivetrain.setPercentOutput(limelightClimber.leftSpeed + .35, limelightClimber.rightSpeed + .35);
					}
					else if(!limelightClimber.limelight.getValidTarget())
					{
						Drivetrain.setPercentOutput(-.55, -.55);;
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
						Drivetrain.setAutoVelocity(600, 600);
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
						Drivetrain.setAutoVelocity(-600, -600);
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
					// System.out.println("1 Hatch Auto DONE!");
					autoStep = 5;
					break;
				case 5:
					if(limelightFourBar.limelight.getValidTarget() && limelightFourBar.y < Constants.limelightYOffset)
					{
						limelightFourBar.center();
						Drivetrain.setPercentOutput(limelightFourBar.leftSpeed - .55, limelightFourBar.rightSpeed - .55);
					}
					else if(!limelightFourBar.limelight.getValidTarget())
					{
						Drivetrain.setPercentOutput(-.55, -.55);;
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
					if(autoTimer.get() < .7)
					{
						HatchGrabber.grabHatch();
						Drivetrain.setAutoVelocity(-600, -600);
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
					if(autoTimer.get() < 0.25)
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
					// System.out.println("Grabbed Second Hatch!");
					grabbedSecondHatch = true;
					autoInitFWD2("StationToLeftRocket");
					limelightFourBar.set(VisionMode.kBlack);
					limelightClimber.set(VisionMode.kClosest);
					autoStep = 10;
					break;
				case 10:
					// System.out.println("Worked?");
					autoStep = 11;
					break;
				case 11:
					if(limelightClimber.limelight.getValidTarget() && limelightClimber.y < Constants.limelightYOffset)
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
					if(autoTimer.get() < 0.35567)
					{
						Drivetrain.setAutoVelocity(600, 600);
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
					if(autoTimer.get() < 0.55)
					{
						HatchGrabber.releaseHatch();
					}
					else
					{
						HatchGrabber.releaseHatch();
						autoStep = 14;
						autoTimer.stop();
						autoTimer.reset();
						autoTimer.start();
					}
					break;
				case 14:
					// System.out.println("2 HATCH AUTOO!!!!!");
					if(autoTimer.get() < 0.1)
					{
						HatchGrabber.releaseHatch();
						Drivetrain.setAutoVelocity(-600, -600);
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
}
