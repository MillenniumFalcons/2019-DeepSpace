package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import frc.team3647StateMachine.SeriesStateMachine;
import frc.team3647autonomous.AutonomousSequences;
import frc.team3647autonomous.Odometry;
import frc.team3647inputs.*;
import frc.team3647subsystems.*;
import frc.team3647subsystems.VisionController.VisionMode;
import frc.team3647utility.AutoChooser;
import frc.team3647utility.PDP;
import frc.team3647utility.Path;
import frc.team3647utility.LastMethod;

public class Robot extends TimedRobot {


	public static PDP pDistributionPanel = new PDP();;
	public static Joysticks mainController = new Joysticks(0);
	public static Joysticks coController = new Joysticks(1);
	public static LastMethod lastMethod;

	public static Gyro gyro = new Gyro();

	public static boolean runAuto = false;

	public static boolean cargoDetection;

	// All the subsystems
	public static BallShooter mBallShooter = BallShooter.getInstance();
	public static HatchGrabber mHatchGrabber = HatchGrabber.getInstance();
	public static BallIntake mBallIntake = BallIntake.getInstance();
	public static MiniShoppingCart mMiniShoppingCart = MiniShoppingCart.getInstance();

	public static Arm mArm = Arm.getInstance();
	public static Elevator mElevator = Elevator.getInstance();
	public static Drivetrain mDrivetrain = Drivetrain.getInstance();

	public static SeriesStateMachine stateMachine = SeriesStateMachine.getInstance();

	private static Notifier autoNotifier;

	private static Notifier drivetrainNotifier = new Notifier(() -> {
		mainController.update();
		mDrivetrain.driveVisionTeleop(mainController, stateMachine, mElevator.getEncoderValue() > 27000);
	});

	private static Notifier armFollowerNotifier = new Notifier(() -> {
		mArm.armNEOFollow();
	});

	private static Notifier pathNotifier = new Notifier(() -> {
		mDrivetrain.updateEncoders();
		Odometry.getInstance().runOdometry(mDrivetrain);
	});

	private static Notifier subsystemsEncodersNotifier = new Notifier(() -> {
		mElevator.updateEncoder();
		mArm.updateEncoder();
	});

	private AutoChooser mAutoChooser;

	private Path mPath;

	@Override
	public void robotInit() {
		LiveWindow.disableAllTelemetry();
		LiveWindow.setEnabled(false);

		VisionController.limelightClimber.set(VisionMode.kBlack);
		VisionController.limelightFourbar.set(VisionMode.kBlack);

		mDrivetrain.init();
		mArm.setToBrake();

		pathNotifier = new Notifier(() -> {
			// AutonomousSequences.frontRocketAuto("Right");
			// AutonomousSequences.sideCargoShipAuto();
			AutonomousSequences.mixedRocketAuto("Left");
		});

		// mAutoChooser = new AutoChooser();
		// mAutoChooser.update();
		// mPath = new Path(mAutoChooser.getSide(), mAutoChooser.getStruct(),
		// mAutoChooser.getMode());

		lastMethod = LastMethod.kStarted;
	}

	@Override
	public void robotPeriodic() {
		if (isAutonomous()) {
			gyro.update();
		}
		// if (!isEnabled()) {
		// mAutoChooser.update();
		// mPath.update(mAutoChooser.getSide(), mAutoChooser.getStruct(),
		// mAutoChooser.getMode());
		// SmartDashboard.putString("Path running first", mPath.getIntialPath());
		// }

		coController.update();
		// System.out.println("Elevator encoder " + mElevator.getEncoderValue());
		// mElevator.updateBannerSensor();
		// if(mElevator.getBannerSensorValue()) {
		// mElevator.resetEncoder();
		// }
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

		runAuto = false;
		AutonomousSequences.autoStep = 0;

		mDrivetrain.init();
		mDrivetrain.setToBrake();
		mDrivetrain.resetEncoders();

		mArm.init();
		mElevator.init();
		stateMachine.init();
		mBallIntake.init();
		mMiniShoppingCart.init();

		// pathNotifier = new Notifier(() -> {
		// mPath.run();
		// });
		// AutonomousSequences.autoInitFWD(mPath.getIntialPath()); // off lvl 2

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

		subsystemsEncodersNotifier.startPeriodic(.05);
		armFollowerNotifier.startPeriodic(.01);

		AirCompressor.run();
		lastMethod = LastMethod.kAuto;
	}

	@Override
	public void autonomousPeriodic() {
		if (!runAuto)
			teleopPeriodic();
		else {
			stateMachine.run();
			mArm.run();
			mElevator.run();
			updateJoysticks();
			if (mainController.buttonB) {
				mDrivetrain.stop();
				runAuto = false;
				disabledInit();
				teleopInit();
			}
		}

	}

	@Override
	public void teleopInit() {
		disableAuto();
		mArm.initSensors();
		mElevator.initSensors();
		mBallIntake.init();
		mMiniShoppingCart.init();

		subsystemsEncodersNotifier.startPeriodic(.05);
		drivetrainNotifier.startPeriodic(.02);
		armFollowerNotifier.startPeriodic(.01);

		AirCompressor.run();
		lastMethod = LastMethod.kTeleop;
	}

	// Teleop Code
	@Override
	public void teleopPeriodic() {
		mHatchGrabber.run(coController);
		stateMachine.updateControllers(cargoDetection, isOperatorControl());
		// subsystems encocers uses notifiers
		stateMachine.run();
		mArm.run();
		mElevator.run();
		mBallShooter.runBlink();

		System.out.println("Elevator encoder " + mElevator.getEncoderValue());
		// Drivetrain uses the notifier
		mMiniShoppingCart.run(mainController);

	}

	private static void disableAuto() {
		mDrivetrain.init();
		mDrivetrain.setToBrake();
		if (autoNotifier != null) {
			autoNotifier.stop();
		}
		if (pathNotifier != null) {
			pathNotifier.stop();
		}
	}

	@Override
	public void disabledInit() {
		drivetrainNotifier.stop();
		mDrivetrain.stop();
		pathNotifier.stop();
		subsystemsEncodersNotifier.stop();
		// armFollowerNotifier.stop();
		// stateMachineRunnerNotifier.stop();
		lastMethod = LastMethod.kDisabled;
	}

	@Override
	public void disabledPeriodic() {

	}

	@Override
	public void testInit() {
		// TestingMethods.reset();
		// mDrivetrain.init();
		mElevator.initSensors();
		// mElevator.initSensors();
		// armFollowerNotifier.startPeriodic(.01);
		lastMethod = LastMethod.kTesting;
		AirCompressor.stop();
	}

	@Override
	public void testPeriodic() {

		// if (mainController.buttonA) {
		// 	mDrivetrain.setRawVelocity(-500, -500);
		// } else if (mainController.buttonY) {
		// 	mDrivetrain.setRawVelocity(500, 500);
		// } else {
		// 	mDrivetrain.driveVisionTeleop(mainController, stateMachine, mainController.rightJoyStickPress);
		// }
		mElevator.getMasterMotor().set(ControlMode.PercentOutput, mainController.leftJoyStickY);
		mainController.update();

		lastMethod = LastMethod.kTesting;
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

}