package frc.robot;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import frc.team3647StateMachine.ArmPosition;
import frc.team3647StateMachine.SeriesStateMachine;
import frc.team3647autonomous.AutonomousSequences;
import frc.team3647autonomous.Odometry;
import frc.team3647autonomous.PathProperties.Side;
import frc.team3647autonomous.Sequences.Rocket;
import frc.team3647autonomous.Sequences.Sequence;
import frc.team3647inputs.*;
import frc.team3647subsystems.*;
import frc.team3647subsystems.VisionController.VisionMode;
import frc.team3647utility.PDP;
import frc.team3647utility.LastMethod;


public class Robot extends TimedRobot {

	public static PDP pDistributionPanel = new PDP();;
	public static Joysticks mainController = new Joysticks(0);
	public static Joysticks coController = new Joysticks(1);
	public static LastMethod lastMethod;

	public static Gyro gyro = new Gyro();

	public static boolean runAuto = false;

	/**
	 * Raw sensor value, use stateMachine.cargoDetectedAfterPiston for value
	 * that checks if the hatch is extended
	 */
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

	
	// private static Sequence leftCargoShipAuto = new CargoShip(Side.LEFT);
	// private static Sequence leftRocket = new Rocket(Side.LEFT);

	private static Notifier autoPathNotifier = new Notifier(() -> {
		// leftRocket.execute();
	});

	private static Notifier drivetrainNotifier = new Notifier(() -> {
		mainController.update();
		mDrivetrain.driveVisionTeleop(mainController, stateMachine, mElevator.getEncoderValue() > 27000, mArm.getEncoderValue());
	});

	private static Notifier odometryDrivetrainNotifier = new Notifier(() -> {
		mDrivetrain.updateEncoders();
		gyro.update();
		Odometry.getInstance().runOdometry(mDrivetrain);
	});

	private static Notifier subsystemsEncodersNotifier = new Notifier(() -> {
		mElevator.updateEncoder();
		mArm.updateEncoder();
	});

	

	@Override
	public void robotInit() {
		LiveWindow.disableAllTelemetry();
		LiveWindow.setEnabled(false);

		VisionController.limelightClimber.set(VisionMode.kBlack);
		VisionController.limelightFourbar.set(VisionMode.kBlack);

		mDrivetrain.init();
		mArm.setToBrake();

		lastMethod = LastMethod.kStarted;
	}

	@Override
	public void robotPeriodic() {
		if (!isEnabled()) {
			VisionController.limelightClimber.setUSBStream();
			VisionController.limelightFourbar.setUSBStream();
		}

		mBallShooter.updateCargoDetection();
		coController.update();

		cargoDetection = mBallShooter.cargoDetection;
	}

	@Override
	public void autonomousInit() {
		try {
			gyro.resetAngle();
		} catch (NullPointerException e) {
			gyro = new Gyro();
		}

		AutonomousSequences.autoStep = 0;
		runAuto = false;
		mDrivetrain.init();
		mDrivetrain.setToBrake();
		mDrivetrain.resetEncoders();

		mArm.init();
		mElevator.init();
		stateMachine.init();
		mBallIntake.init();
		mHatchGrabber.init();



		if (!runAuto) {
			drivetrainNotifier.startPeriodic(.02);
		} else {
			odometryDrivetrainNotifier.startPeriodic(.02);
			autoPathNotifier.startPeriodic(.02);
			// AutoTest.init("StraightAndTurnRight");
		}

		subsystemsEncodersNotifier.startPeriodic(.05);

		AirCompressor.run();

		lastMethod = LastMethod.kAuto;
	}

	@Override
	public void autonomousPeriodic() {
		if (!runAuto)
			teleopPeriodic();
		else {
			// Sequence.log("Odometry: " + Odometry.getInstance());
			stateMachine.run();
			mArm.run();
			mElevator.run();
			updateJoysticks();
			if (mainController.buttonB) {
				mDrivetrain.stop();
				mDrivetrain.setOpenLoop(0, 0);
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
		mHatchGrabber.init();
		subsystemsEncodersNotifier.startPeriodic(.05);
		drivetrainNotifier.startPeriodic(.02);

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
		mBallShooter.runBlink(stateMachine.cargoDetectedAfterPiston);

		// Drivetrain uses the notifier
	}

	private static void disableAuto() {
		mDrivetrain.init();
		mDrivetrain.setToBrake();
		if (autoPathNotifier != null) {
			autoPathNotifier.close();
		}
		if (odometryDrivetrainNotifier != null) {
			odometryDrivetrainNotifier.close();
		}
	}

	@Override
	public void disabledInit() {
		drivetrainNotifier.stop();
		mDrivetrain.stop();
		odometryDrivetrainNotifier.stop();
		if(autoPathNotifier != null) {
			autoPathNotifier.stop();
		}
		subsystemsEncodersNotifier.stop();
		// armFollowerNotifier.stop();
		lastMethod = LastMethod.kDisabled;
	}

	@Override
	public void disabledPeriodic() {
		VisionController.limelightClimber.setLED();
		VisionController.limelightFourbar.setLED();
		mElevator.stop();
		// mArm.stop();
		mArm.stop();
		mHatchGrabber.stop();
		mDrivetrain.stop();
	}

	
	@Override
	public void testInit() {
		TestingMethods.reset();

		// mArm.setEncoderValue(-13050);
		// AirCompressor.run();
		mArm.init();
		lastMethod = LastMethod.kTesting;
		AirCompressor.stop();
		
	}

	@Override
	public void testPeriodic() {
		// mainController.update();
		TestingMethods.test(mArm);
		// mDrivetrain.driveVisionTeleop(mainController, stateMachine, mElevator.getEncoderValue() > 27000, mArm.getEncoderValue());
		// if(mainController.leftBumper){mHatchGrabber.grabHatch();}
		// else {mHatchGrabber.stop();}

		
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
}