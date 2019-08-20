package frc.team3647autonomous;

import edu.wpi.first.wpilibj.Timer;
import jaci.pathfinder.Trajectory;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.team3647subsystems.VisionController;
import frc.team3647subsystems.VisionController.VisionMode;
import frc.team3647utility.Units;
import frc.team3647StateMachine.RobotState;

public class AutonomousSequences {
	private static VisionController limelightFourbar = VisionController.limelightFourbar;
	private static VisionController limelightClimber = VisionController.limelightClimber;

	public static int autoStep = 0;
	private static Timer autoTimer = new Timer();

	public static DriveSignal driveSignal;
	public static Trajectory trajectory;
	public static RamseteFollower ramseteFollower;
	public static Trajectory.Segment current;
	public static double leftSpeed;
	public static double rightSpeed;
	public static double linearVelocity;
	public static double visionVelocityConstant = 2500;

	// private static Drivetrain mDrivetrain = Robot.mDrivtrain;

	public static void autoInitFWD(String trajectoryName) {
		Robot.mDrivetrain.selectPIDF(Constants.velocitySlotIdx, Constants.rightVelocityPIDF, Constants.leftVelocityPIDF);
		driveSignal = new DriveSignal();
		trajectory = TrajectoryUtil.getTrajectoryFromName(trajectoryName);
		ramseteFollower = new RamseteFollower(trajectory, MotionProfileDirection.FORWARD);
		Odometry.getInstance().setInitialOdometry(trajectory);
		Odometry.getInstance().odometryInit();
		autoTimer.stop();
		autoTimer.reset();
		// autoTimer.start();
		autoStep = 0;
	}

	public static void autoInitFWD2(String trajectoryName) {
		Robot.mDrivetrain.selectPIDF(Constants.velocitySlotIdx, Constants.rightVelocityPIDF, Constants.leftVelocityPIDF);
		driveSignal = new DriveSignal();
		trajectory = TrajectoryUtil.getTrajectoryFromName(trajectoryName);
		ramseteFollower = new RamseteFollower(trajectory, MotionProfileDirection.FORWARD);
	}

	public static void autoInitBWD(String trajectoryName) {
		Robot.mDrivetrain.selectPIDF(Constants.velocitySlotIdx, Constants.rightVelocityPIDF, Constants.leftVelocityPIDF);
		driveSignal = new DriveSignal();
		trajectory = TrajectoryUtil.getTrajectoryFromName(trajectoryName);
		ramseteFollower = new RamseteFollower(trajectory, MotionProfileDirection.BACKWARD);
	}

	public static void testing() {
		driveSignal = ramseteFollower.getNextDriveSignal();
		current = ramseteFollower.currentSegment();

		rightSpeed = Units.metersToEncoderTicks(driveSignal.getRight() / 10);
		leftSpeed = Units.metersToEncoderTicks(driveSignal.getLeft() / 10);

		if (ramseteFollower.isFinished()) {
			ramseteFollower = new RamseteFollower(trajectory, MotionProfileDirection.BACKWARD);
			Odometry.getInstance().setInitialOdometry(TrajectoryUtil.reversePath(trajectory));
		}
	}

	public static void ramsetePeriodic() {
		driveSignal = ramseteFollower.getNextDriveSignal();
		current = ramseteFollower.currentSegment();
		rightSpeed = Units.metersToEncoderTicks(driveSignal.getRight() / 10);
		leftSpeed = Units.metersToEncoderTicks(driveSignal.getLeft() / 10);
		linearVelocity = Units.metersToEncoderTicks(ramseteFollower.linearVelocity / 10);
	}

	public static void runVision() {
		ramsetePeriodic();
		if (!ramseteFollower.pathFractionSegment(.5) && !ramseteFollower.isFinished()) {
			Robot.mDrivetrain.setAutoVelocity(leftSpeed, rightSpeed);
		} else if (ramseteFollower.pathFractionSegment(.5)) {
			limelightClimber.center();
			Robot.mDrivetrain.setAutoVelocity(leftSpeed + (visionVelocityConstant) * limelightClimber.leftSpeed,
					rightSpeed + (visionVelocityConstant) * limelightClimber.rightSpeed);
		} else if (ramseteFollower.isFinished() && limelightClimber.y > Constants.limelightYOffset) {
			limelightClimber.center();
			Robot.mDrivetrain.setAutoVelocity((visionVelocityConstant) * limelightClimber.leftSpeed,
					(visionVelocityConstant) * limelightClimber.rightSpeed);
		} else {
			Robot.mHatchGrabber.releaseHatch();
			Robot.mDrivetrain.setAutoVelocity(-500, -500);
		}
	}

	public static void runPath() {
		ramsetePeriodic();
		Robot.mDrivetrain.setAutoVelocity(leftSpeed, rightSpeed);
	}

	public static void frontRocketAuto(String LeftOrRight) {
		ramsetePeriodic();
		switch (autoStep) {
		case 0:
			if (!ramseteFollower.pathFractionSegment(.6) && !ramseteFollower.isFinished()) {
				Robot.mDrivetrain.setAutoVelocity(leftSpeed, rightSpeed);
			} else {
				if (LeftOrRight.equals("Left")) {
					limelightClimber.set(VisionMode.kLeft);
				} else {
					limelightClimber.set(VisionMode.kRight);
				}
				autoStep = 1;
			}
			break;
		case 1:
			if (ramseteFollower.pathFractionSegment(.6))
				Robot.stateMachine.aimedRobotState = RobotState.HATCHL1FORWARDS;
			if (ramseteFollower.pathFractionSegment(.5)) {
				limelightClimber.center();
				Robot.mDrivetrain.setAutoVelocity(linearVelocity + (visionVelocityConstant) * limelightClimber.leftSpeed,
						linearVelocity + (visionVelocityConstant) * limelightClimber.rightSpeed);
			} else if (ramseteFollower.isFinished()) {
				Robot.mHatchGrabber.releaseHatch();
				autoStep = 4;
			}
			break;
		case 4:
			autoInitBWD(LeftOrRight + "FrontRocketToStation");
			limelightClimber.set(VisionMode.kBlack);
			limelightFourbar.set(VisionMode.kRight);
			autoStep = 5;
			break;
		case 5:
			if (!ramseteFollower.pathFractionSegment(.5) && !ramseteFollower.isFinished()) {
				Robot.mDrivetrain.setAutoVelocity(leftSpeed, rightSpeed);
				if (ramseteFollower.pathFractionSegment(.15)) {
					Robot.stateMachine.aimedRobotState = RobotState.HATCHL1BACKWARDS;
					Robot.mHatchGrabber.stop();
				}
			} else {
				autoStep = 6;
			}
			break;
		case 6:
			if (ramseteFollower.pathFractionSegment(.5)) {
				limelightFourbar.center();
				Robot.mHatchGrabber.grabHatch();
				Robot.mDrivetrain.setAutoVelocity(linearVelocity + (visionVelocityConstant) * limelightFourbar.leftSpeed,
						linearVelocity + (visionVelocityConstant) * limelightFourbar.rightSpeed);
			} else {
				autoInitFWD2("StationTo" + LeftOrRight + "FrontRocket");
				limelightFourbar.set(VisionMode.kBlack);

				if (LeftOrRight.equals("Left")) {
					limelightClimber.set(VisionMode.kLeft);
				} else {
					limelightClimber.set(VisionMode.kRight);
				}

				autoStep = 10;
			}
			break;
		case 10:
			if (!ramseteFollower.pathFractionSegment(.5) && !ramseteFollower.isFinished()) {
				Robot.mDrivetrain.setAutoVelocity(leftSpeed, rightSpeed);
				if (ramseteFollower.pathFractionSegment(.2)) {
					Robot.stateMachine.aimedRobotState = RobotState.HATCHL2FORWARDS;
					Robot.mHatchGrabber.runConstant();
				}
			} else {
				autoStep = 11;
			}
			break;
		case 11:
			if (ramseteFollower.pathFractionSegment(.5)) {
				limelightClimber.center();
				Robot.mDrivetrain.setAutoVelocity(linearVelocity + (visionVelocityConstant) * limelightClimber.leftSpeed,
						linearVelocity + (visionVelocityConstant) * limelightClimber.rightSpeed);
			} else if (ramseteFollower.isFinished()) {
				Robot.mHatchGrabber.releaseHatch();
				autoStep = 13;
			}
			break;
		case 13:
			if (autoTimer.get() < 0.3) {
				Robot.mHatchGrabber.releaseHatch();
				Robot.mDrivetrain.setAutoVelocity(-500, -500);
			} else {
				Robot.mHatchGrabber.stop();
				// Drivetrain.stop();
				autoStep = 14;
				autoTimer.stop();
				autoTimer.reset();
			}
			break;
		case 14:
			Robot.runAuto = false;
			Robot.mDrivetrain.stop();
			Robot.mHatchGrabber.stop();
			break;
		default:
			Robot.mDrivetrain.stop();
			break;
		}
	}

	public static void mixedRocketAuto(String LeftOrRight) {
		ramsetePeriodic();
		switch (autoStep) {
		case 0:
			if (ramseteFollower.pathFractionSegment(.65)) {
				Robot.stateMachine.aimedRobotState = RobotState.HATCHL2FORWARDS;
			}
			if (!ramseteFollower.pathFractionSegment(.9) && !ramseteFollower.isFinished()) {
				Robot.mDrivetrain.setAutoVelocity(leftSpeed, rightSpeed);
			} else {
				if (LeftOrRight.equals("Left")) {
					limelightClimber.set(VisionMode.kRight);
				} else {
					limelightClimber.set(VisionMode.kLeft);
				}
				autoStep = 1;
			}
			break;
		case 1:
			if (ramseteFollower.pathFractionSegment(.8)) {
				limelightClimber.center();
				Robot.mDrivetrain.setAutoVelocity(linearVelocity + (visionVelocityConstant) * limelightClimber.leftSpeed,
						linearVelocity + (visionVelocityConstant) * limelightClimber.rightSpeed);
			} else if (ramseteFollower.isFinished()) {
				Robot.mHatchGrabber.releaseHatch();
				autoStep = 4;
			}
			break;
		case 4:
			autoInitBWD(LeftOrRight + "BackRocketToStation");
			limelightClimber.set(VisionMode.kBlack);
			if (LeftOrRight.equals("Left")) {
				limelightFourbar.set(VisionMode.kRight);
			} else {
				limelightFourbar.set(VisionMode.kLeft);
			}
			autoStep = 5;
			break;
		case 5:
			if (!ramseteFollower.pathFractionSegment(.6) && !ramseteFollower.isFinished()) {
				Robot.mDrivetrain.setAutoVelocity(leftSpeed, rightSpeed);
				if (ramseteFollower.pathFractionSegment(.15)) {
					Robot.stateMachine.aimedRobotState = RobotState.HATCHL1BACKWARDS;
					Robot.mHatchGrabber.stop();
				}
			} else {
				autoStep = 6;
			}
			break;
		case 6:
			if (ramseteFollower.pathFractionSegment(.6)) {
				limelightFourbar.center();
				Robot.mHatchGrabber.grabHatch();
				Robot.mDrivetrain.setAutoVelocity(linearVelocity + (visionVelocityConstant) * limelightFourbar.leftSpeed,
						linearVelocity + (visionVelocityConstant) * limelightFourbar.rightSpeed);
			} else {
				autoInitFWD2("StationTo" + LeftOrRight + "FrontRocket");
				limelightFourbar.set(VisionMode.kBlack);
				if (LeftOrRight.equals("Left")) {
					limelightClimber.set(VisionMode.kLeft);
				} else {
					limelightClimber.set(VisionMode.kRight);
				}
				autoStep = 10;
			}
			break;
		case 10:
			if (!ramseteFollower.pathFractionSegment(.6) && !ramseteFollower.isFinished()) {
				Robot.mDrivetrain.setAutoVelocity(leftSpeed, rightSpeed);
				if (ramseteFollower.pathFractionSegment(.2)) {
					Robot.stateMachine.aimedRobotState = RobotState.HATCHL2FORWARDS;
					Robot.mHatchGrabber.runConstant();
				}
			} else {
				autoStep = 11;
			}
			break;
		case 11:
			if (ramseteFollower.pathFractionSegment(.6)) {
				limelightClimber.center();
				Robot.mDrivetrain.setAutoVelocity(linearVelocity + (visionVelocityConstant) * limelightClimber.leftSpeed,
						linearVelocity + (visionVelocityConstant) * limelightClimber.rightSpeed);
			} else if (ramseteFollower.isFinished()) {
				Robot.mHatchGrabber.releaseHatch();
				autoStep = 13;
			}
			break;
		case 13:
			if (autoTimer.get() < 0.3) {
				Robot.mHatchGrabber.releaseHatch();
				Robot.mDrivetrain.setAutoVelocity(-500, -500);
			} else {
				Robot.mHatchGrabber.stop();
				// Drivetrain.stop();
				autoStep = 14;
				autoTimer.stop();
				autoTimer.reset();
			}
			break;
		case 14:
			Robot.runAuto = false;
			Robot.mDrivetrain.stop();
			Robot.mHatchGrabber.stop();
			break;
		default:
			Robot.mDrivetrain.stop();
			break;
		}
	}

	public static void sideCargoShipAuto(String LeftOrRight) {
		ramsetePeriodic();
		switch (autoStep) {
		case 0:
			if (LeftOrRight.equals("Left")) {
				limelightClimber.set(VisionMode.kRight);
			} else {
				limelightClimber.set(VisionMode.kLeft);
			}
			limelightClimber.set(VisionMode.kRight);
			if (!ramseteFollower.pathFractionSegment(.8) && !ramseteFollower.isFinished()) {
				Robot.mDrivetrain.setAutoVelocity(leftSpeed, rightSpeed);
			} else {
				autoStep = 1;
			}
			break;
		case 1:
			if (ramseteFollower.pathFractionSegment(.8)) {
				Robot.stateMachine.aimedRobotState = RobotState.HATCHL1FORWARDS;
				limelightClimber.center();
				Robot.mDrivetrain.setAutoVelocity(linearVelocity + (visionVelocityConstant) * limelightClimber.leftSpeed,
						linearVelocity + (visionVelocityConstant) * limelightClimber.rightSpeed);
			} else if (ramseteFollower.isFinished()) {
				Robot.mHatchGrabber.releaseHatch();
				autoStep = 4;
			}
			break;
		case 4:
			autoInitBWD(LeftOrRight + "CargoShipBay1ToStation");
			limelightClimber.set(VisionMode.kBlack);
			if (LeftOrRight.equals("Left")) {
				limelightClimber.set(VisionMode.kRight);
			} else {
				limelightClimber.set(VisionMode.kLeft);
			}
			autoStep = 5;
			break;
		case 5:
			if (!ramseteFollower.pathFractionSegment(.6) && !ramseteFollower.isFinished()) {
				Robot.mDrivetrain.setAutoVelocity(leftSpeed, rightSpeed);
				if (ramseteFollower.pathFractionSegment(.15)) {
					Robot.stateMachine.aimedRobotState = RobotState.HATCHL1BACKWARDS;
					Robot.mHatchGrabber.runConstant();
				}
			} else {
				autoStep = 6;
			}
			break;
		case 6:
			if (ramseteFollower.pathFractionSegment(.6)) {
				limelightFourbar.center();
				Robot.mHatchGrabber.grabHatch();
				Robot.mDrivetrain.setAutoVelocity(linearVelocity + (visionVelocityConstant) * limelightFourbar.leftSpeed,
						linearVelocity + (visionVelocityConstant) * limelightFourbar.rightSpeed);
			} else {
				autoInitFWD2("StationTo" + LeftOrRight + "CargoShipBay2");
				limelightFourbar.set(VisionMode.kBlack);
				if (LeftOrRight.equals("Left")) {
					limelightClimber.set(VisionMode.kRight);
				} else {
					limelightClimber.set(VisionMode.kLeft);
				}
				autoStep = 10;
			}
			break;
		case 10:
			if (!ramseteFollower.pathFractionSegment(.82) && !ramseteFollower.isFinished()) {
				Robot.mDrivetrain.setAutoVelocity(leftSpeed, rightSpeed);
				if (ramseteFollower.pathFractionSegment(.2)) {
					Robot.stateMachine.aimedRobotState = RobotState.HATCHL1FORWARDS;
					Robot.mHatchGrabber.runConstant();
				}
			} else {
				autoStep = 11;
			}
			break;
		case 11:
			if (ramseteFollower.pathFractionSegment(.82)) {
				limelightClimber.center();
				Robot.mDrivetrain.setAutoVelocity(linearVelocity + (visionVelocityConstant) * limelightClimber.leftSpeed,
						linearVelocity + (visionVelocityConstant) * limelightClimber.rightSpeed);
			} else if (ramseteFollower.isFinished()) {
				Robot.mHatchGrabber.releaseHatch();
				autoStep = 13;
			}
			break;
		case 13:
			if (autoTimer.get() < 0.3) {
				Robot.mHatchGrabber.releaseHatch();
				Robot.mDrivetrain.setAutoVelocity(-500, -500);
			} else {
				Robot.mHatchGrabber.stop();
				Robot.mDrivetrain.stop();
				autoStep = 14;
				autoTimer.stop();
				autoTimer.reset();
			}
			break;
		case 14:
			Robot.runAuto = false;
			Robot.mDrivetrain.stop();
			Robot.mHatchGrabber.stop();
			break;
		default:
			Robot.mDrivetrain.stop();
			break;
		}
	}

	public static void mixedCargoShipAuto(String LeftOrRight) {
		ramsetePeriodic();
		switch (autoStep) {
		case 0:
			if (LeftOrRight.equals("Left")) {
				limelightClimber.set(VisionMode.kLeft);
			} else {
				limelightClimber.set(VisionMode.kRight);
			}
			if (!ramseteFollower.pathFractionSegment(.8) && !ramseteFollower.isFinished()) {
				Robot.mDrivetrain.setAutoVelocity(leftSpeed, rightSpeed);
			} else {
				autoStep = 1;
			}
			break;
		case 1:
			if (ramseteFollower.pathFractionSegment(.8)) {
				Robot.stateMachine.aimedRobotState = RobotState.HATCHL1FORWARDS;
				limelightClimber.center();
				Robot.mDrivetrain.setAutoVelocity(linearVelocity + (visionVelocityConstant) * limelightClimber.leftSpeed,
						linearVelocity + (visionVelocityConstant) * limelightClimber.rightSpeed);
			} else if (ramseteFollower.isFinished()) {
				Robot.mHatchGrabber.releaseHatch();
				autoStep = 4;
			}
			break;
		case 4:
			autoInitBWD("Middle" + LeftOrRight + "CargoShipToStation");
			limelightClimber.set(VisionMode.kBlack);
			if (LeftOrRight.equals("Left")) {
				limelightClimber.set(VisionMode.kRight);
			} else {
				limelightClimber.set(VisionMode.kLeft);
			}
			autoStep = 5;
			break;
		case 5:
			if (!ramseteFollower.pathFractionSegment(.6) && !ramseteFollower.isFinished()) {
				Robot.mDrivetrain.setAutoVelocity(leftSpeed, rightSpeed);
				if (ramseteFollower.pathFractionSegment(.15)) {
					Robot.stateMachine.aimedRobotState = RobotState.HATCHL1BACKWARDS;
					Robot.mHatchGrabber.runConstant();
				}
			} else {
				autoStep = 6;
			}
			break;
		case 6:
			if (ramseteFollower.pathFractionSegment(.6)) {
				limelightFourbar.center();
				Robot.mHatchGrabber.grabHatch();
				Robot.mDrivetrain.setAutoVelocity(linearVelocity + (visionVelocityConstant) * limelightFourbar.leftSpeed,
						linearVelocity + (visionVelocityConstant) * limelightFourbar.rightSpeed);
			} else {
				autoInitFWD2("StationTo" + LeftOrRight + "CargoShipBay2");
				limelightFourbar.set(VisionMode.kBlack);
				if (LeftOrRight.equals("Left")) {
					limelightClimber.set(VisionMode.kRight);
				} else {
					limelightClimber.set(VisionMode.kLeft);
				}
				autoStep = 10;
			}
			break;
		case 10:
			if (!ramseteFollower.pathFractionSegment(.82) && !ramseteFollower.isFinished()) {
				Robot.mDrivetrain.setAutoVelocity(leftSpeed, rightSpeed);
				if (ramseteFollower.pathFractionSegment(.2)) {
					Robot.stateMachine.aimedRobotState = RobotState.HATCHL1FORWARDS;
					Robot.mHatchGrabber.runConstant();
				}
			} else {
				autoStep = 11;
			}
			break;
		case 11:
			if (ramseteFollower.pathFractionSegment(.82)) {
				limelightClimber.center();
				Robot.mDrivetrain.setAutoVelocity(linearVelocity + (visionVelocityConstant) * limelightClimber.leftSpeed,
						linearVelocity + (visionVelocityConstant) * limelightClimber.rightSpeed);
			} else if (ramseteFollower.isFinished()) {
				Robot.mHatchGrabber.releaseHatch();
				autoTimer.stop();
				autoTimer.reset();
				autoStep = 13;
			}
			break;
		case 13:
			if (autoTimer.get() < 0.3) {
				Robot.mHatchGrabber.releaseHatch();
				Robot.mDrivetrain.setAutoVelocity(-500, -500);
			} else {
				Robot.mHatchGrabber.stop();
				Robot.mDrivetrain.stop();
				autoStep = 14;
				autoTimer.stop();
				autoTimer.reset();
			}
			break;
		case 14:
			Robot.runAuto = false;
			Robot.mDrivetrain.stop();
			Robot.mHatchGrabber.stop();
			break;
		default:
			Robot.mDrivetrain.stop();
			break;
		}

	}

}
