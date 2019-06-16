package frc.robot;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3647StateMachine.RobotState;
import frc.team3647StateMachine.SeriesStateMachine;
import frc.team3647autonomous.AutonomousSequences;
import frc.team3647autonomous.Odometry;
import frc.team3647inputs.*;
import frc.team3647subsystems.*;
import frc.team3647subsystems.VisionController.VisionMode;
import frc.team3647utility.AutoChooser;
import frc.team3647utility.PDP;
import frc.team3647utility.Path;

public class Robot extends TimedRobot {

	public static PDP pDistributionPanel;
	public static Joysticks mainController;
	public static Joysticks coController;

	public static Gyro gyro;

	private static Notifier drivetrainNotifier;
	private static Notifier armFollowerNotifier;
	private static Notifier autoNotifier;
	private static Notifier pathNotifier;
	private static Notifier subsystemsEncodersNotifier;;

	public static boolean runAuto = false;

	public static boolean cargoDetection;


	// All the subsystems
	public static BallShooter mBallShooter = BallShooter.getInstance();
	public static HatchGrabber mHatchGrabber = HatchGrabber.getInstance();
	public static BallIntake mBallIntake = BallIntake.getInstance();
	public static MiniShoppingCart mMiniShoppingCart = MiniShoppingCart.getInstance();

	public static Arm mArm = Arm.getInstance();
	public static Elevator mElevator = Elevator.getInstance();
	public static Drivetrain mDrivtrain = Drivetrain.getInstance();

	public static SeriesStateMachine stateMachine = SeriesStateMachine.getInstance();

	private AutoChooser mAutoChooser;

	private Path mPath;

	@Override
	public void robotInit() {
		LiveWindow.disableAllTelemetry();
		LiveWindow.setEnabled(false);
		pDistributionPanel = new PDP();

		VisionController.limelightClimber.set(VisionMode.kBlack);
		VisionController.limelightFourbar.set(VisionMode.kBlack);
		mainController = new Joysticks(0);
		coController = new Joysticks(1);
		gyro = new Gyro();

		mDrivtrain.init();

		mArm.setToBrake();

		subsystemsEncodersNotifier = new Notifier(() -> {
			mElevator.updateEncoder();
			mArm.updateEncoder();
		});
		armFollowerNotifier = new Notifier(() -> {
			mArm.armNEOFollow();
		});

		drivetrainNotifier = new Notifier(() -> {
			mainController.update();
			driveVisionTeleop();
		});

		autoNotifier = new Notifier(() -> {
			mDrivtrain.updateEncoders();
			Odometry.getInstance().runOdometry(mDrivtrain);
		});

		pathNotifier = new Notifier(() -> {
			// AutonomousSequences.frontRocketAuto("Right");
			// AutonomousSequences.sideCargoShipAuto();
			AutonomousSequences.mixedRocketAuto("Left");
		});

		mAutoChooser = new AutoChooser();
		mAutoChooser.update();
		mPath = new Path(mAutoChooser.getSide(), mAutoChooser.getStruct(), mAutoChooser.getMode());
	}

	@Override
	public void robotPeriodic() {
		if (isAutonomous()) {
			gyro.update();
		}
		if (isEnabled()) {
			coController.update();
		} else {
			mAutoChooser.update();
			mPath.update(mAutoChooser.getSide(), mAutoChooser.getStruct(), mAutoChooser.getMode());
			SmartDashboard.putString("Path running first", mPath.getIntialPath());
		}
		// mainController.update(); Done in drivetrain notifier!
		// SmartDashboard.putNumber("Match Timer",
		// DriverStation.getInstance().getMatchTime());
		cargoDetection = mBallShooter.cargoDetection();
	}

	@Override
	public void autonomousInit() {
		try {
			gyro.resetAngle();
		} catch (NullPointerException e) {
			gyro = new Gyro();
		}

		runAuto = true;
		AutonomousSequences.autoStep = 0;

		mDrivtrain.init();
		mDrivtrain.setToBrake();
		mDrivtrain.resetEncoders();

		mArm.init();
		mElevator.init();
		stateMachine.init();
		mBallIntake.init();
		mMiniShoppingCart.init();

		pathNotifier = new Notifier(() -> {
			mPath.run();
		});
		AutonomousSequences.autoInitFWD(mPath.getIntialPath()); // off lvl 2

		// AutonomousSequences.autoInitFWD("LeftPlatformToBackLeftRocket"); //off lvl 1
		// AutonomousSequences.autoInitFWD("LeftPlatformToBackLeftRocket"); //mixed left
		// rocket
		// AutonomousSequences.autoInitFWD("PlatformToLeftMiddleLeftCargoShip");
		// //cargoship left
		// AutonomousSequences.autoInitFWD("RightPlatformToRightRocket"); //right Rocket
		// AutonomousSequences.autoInitFWD("RightPlatformToBackRightRocket"); //right
		// Rocket
		if (!runAuto) {
			drivetrainNotifier.startPeriodic(.02);
		} else {
			pathNotifier.startPeriodic(.02);
			autoNotifier.startPeriodic(.01);
		}

		armFollowerNotifier.startPeriodic(.01);
		subsystemsEncodersNotifier.startPeriodic(.05);

		AirCompressor.run();
	}

	@Override
	public void autonomousPeriodic() {

		if (mainController.buttonB) {
			mDrivtrain.stop();
			runAuto = false;
			disabledInit();
			teleopInit();
		}

		if (!runAuto)
			teleopPeriodic();
		else {
			stateMachine.run();
			mArm.run();
			mElevator.run();
			updateJoysticks();
		}
	}

	@Override
	public void teleopInit() {
		disableAuto();
		mArm.initSensors();
		mElevator.initSensors();
		mBallIntake.init();
		mMiniShoppingCart.init();

		drivetrainNotifier.startPeriodic(.02);
		armFollowerNotifier.startPeriodic(.01);

		subsystemsEncodersNotifier.startPeriodic(.02);
		AirCompressor.run();
	}

	// Teleop Code
	@Override
	public void teleopPeriodic() {
		mHatchGrabber.run(coController);
		stateMachine.updateControllers(cargoDetection);
		// subsystems encocers uses notifiers
		stateMachine.run();
		mArm.run();
		mElevator.run();
		mBallShooter.runBlink();

		// Drivetrain uses the notifier

		if (stateMachine.runClimberManually) {
			mMiniShoppingCart.run(mainController);
		}

	}

	private static void disableAuto() {
		mDrivtrain.init();
		mDrivtrain.setToBrake();
		autoNotifier.stop();
		pathNotifier.stop();
	}

	@Override
	public void disabledInit() {
		drivetrainNotifier.stop();
		mDrivtrain.stop();
		pathNotifier.stop();
		subsystemsEncodersNotifier.stop();
		// armFollowerNotifier.stop();
		// stateMachineRunnerNotifier.stop();
	}

	@Override
	public void disabledPeriodic() {

	}

	@Override
	public void testInit() {
		// Elevator.init();
		// drivetrainNotifier.startPeriodic(.02);
		mMiniShoppingCart.init();
		// Arm.init();
		// armFollowerNotifier.startPeriodic(.01);
		// Arm.initSensors();
		// drivetrainNotifier.startPeriodic(.02);
	}

	@Override
	public void testPeriodic() {
		mMiniShoppingCart.run(mainController);
		mainController.update();

		// BallShooter.stopMotor();
		// HatchGrabber.stopMotor();
		// AirCompressor.run();
	}

	private void updateJoysticks() {
		try {
			mainController.update();
		} catch (NullPointerException e) {
			System.out.println(e);
			mainController = new Joysticks(0);
		} catch (Exception e) {
			System.out.println(e);
		}

		try {
			coController.update();
		} catch (NullPointerException e) {
			System.out.println(e);
			coController = new Joysticks(1);
		} catch (Exception e) {
			System.out.println(e);
		}
	}

	double threshold = Constants.limelightThreshold;

	private void driveVisionTeleop() {
		if (mainController.rightBumper && mainController.rightJoyStickX < .1) {
			vision((mElevator.getEncoderValue() > 27000), VisionMode.kClosest);
		} else {
			VisionController.limelightClimber.set(VisionMode.kBlack);
			VisionController.limelightFourbar.set(VisionMode.kBlack);
			if (mElevator.getEncoderValue() > 27000)
				mDrivtrain.customArcadeDrive(mainController.rightJoyStickX * .65, mainController.leftJoyStickY * .6);
			else
				mDrivtrain.customArcadeDrive(mainController.rightJoyStickX, mainController.leftJoyStickY);
		}
	}

	private void vision(boolean scaleJoy, VisionMode mode) {
		double joyY = mainController.leftJoyStickY;
		double joyX = mainController.rightJoyStickX;

		VisionController mVision = getLimelight();

		VisionController mVisionDriver = mVision;
		if (mVision.equals(VisionController.limelightClimber))
			mVisionDriver = VisionController.limelightFourbar;
		else
			mVisionDriver = VisionController.limelightClimber;

		if (scaleJoy) {
			joyY *= .6;
			joyX *= .65;
		}

		mVision.set(mode);
		if (mode != VisionMode.kBlack) {
			mVisionDriver.set(VisionMode.kDriver);
		}
		if (mode == VisionMode.kDriver) {
			mDrivtrain.customArcadeDrive(joyX, joyY);
		} else {
			mVision.center();
			mDrivtrain.setOpenLoop(mVision.leftSpeed + joyY, mVision.rightSpeed + joyY);
		}
	}

	private VisionController getLimelight() {
		if (mArm.aimedState != null) {
			// Arm is flipped bwds
			if (mArm.aimedState.getValue() < Constants.armSRXVerticalStowed) {
				// If cargo then its actually forwards
				if (!cargoDetection || RobotState.CARGOLOADINGSTATIONFWD.equals(stateMachine.aimedRobotState))
					return VisionController.limelightClimber;
				else
					return VisionController.limelightFourbar;
			}
			// Arm is forwards
			else {
				// if cargo its actually backwards
				if (!cargoDetection || RobotState.CARGOLOADINGSTATIONBWD.equals(stateMachine.aimedRobotState))
					return VisionController.limelightFourbar;
				else
					return VisionController.limelightClimber;
			}
		}
		return VisionController.limelightClimber;
	}
}