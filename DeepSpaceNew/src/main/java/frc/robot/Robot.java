package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import frc.team3647autonomous.AutonomousSequences;
import frc.team3647autonomous.DriveSignal;
import frc.team3647autonomous.MotionProfileDirection;
import frc.team3647autonomous.Odometry;
import frc.team3647autonomous.RamseteFollower;
import frc.team3647autonomous.TrajectoryUtil;
import frc.team3647inputs.*;
import frc.team3647subsystems.*;
import frc.team3647subsystems.Elevator.ElevatorLevel;
import frc.team3647utility.Units;
import jaci.pathfinder.Trajectory;

public class Robot extends TimedRobot 
{
	// private static final String kDefaultAuto = "Default";
	// private static final String kCustomAuto = "My Auto";
	// private String m_autoSelected;
	// private final SendableChooser<String> m_chooser = new SendableChooser<>();

	public static Joysticks mainController;
	public static Joysticks coController;
	public static Gyro gyro;

	public static double[] PIDFleft = new double[4];
	public static double[] PIDFright = new double[4];
	public static double[] PIDF = new double[4];
	public static int mVel = 0;
	public static int mAccel = 0;
	private Timer autoTimer;

	@Override
	public void robotInit() 
	{
		// m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
		// m_chooser.addOption("My Auto", kCustomAuto);
		// SmartDashboard.putData("Auto choices", m_chooser);

		mainController = new Joysticks(0);
		coController = new Joysticks(1);
		gyro = new Gyro();
		// TestFunctions.shuffleboardInit();
		// Drivetrain.drivetrainInitialization();

		Drivetrain.initializeSmartDashboardVelAccel();
		Arm.armNEO.setIdleMode(IdleMode.kBrake);
		autoTimer = new Timer();
	}

	@Override
	public void robotPeriodic() 
	{
		gyro.updateGyro();
		updateJoysticks();
	}

	DriveSignal driveSignal;
	Trajectory trajectory;
	RamseteFollower ramseteFollower;
	Trajectory.Segment current;
	

	@Override
	public void autonomousInit() 
	{
		gyro.resetAngle();
		Drivetrain.drivetrainInitialization();
		Drivetrain.resetEncoders();
		AirCompressor.runCompressor();
		SeriesStateMachine.seriesStateMachineInit();
		Arm.armInitialization();
		Elevator.elevatorInitialization();
		// driveSignal = new DriveSignal();
		// trajectory = TrajectoryUtil.getTrajectoryFromName("Level2RightToCargoRight");
		// ramseteFollower = new RamseteFollower(trajectory,
		// MotionProfileDirection.FORWARD);
		// Odometry.getInstance().setInitialOdometry(trajectory);
		// Odometry.getInstance().odometryInit();
		// ranBackwardsOnce = false;
		AutonomousSequences.autoInit("Level2RightToCargoRight");
		autoTimer.reset();
		autoTimer.start();

	}


	@Override
	public void autonomousPeriodic() 
	{
		if(autoTimer.get() > 2)
		{
			SeriesStateMachine.runSeriesStateMachine();
			Elevator.runElevator();
			Arm.runArm();
		}
		AutonomousSequences.level2RightToCargoShipRight();
		// driveSignal = ramseteFollower.getNextDriveSignal();
		// current = ramseteFollower.currentSegment();

		// right = Units.metersToEncoderTicks(driveSignal.getRight() / 20);
		// left = Units.metersToEncoderTicks(driveSignal.getLeft() / 20);
		// Drivetrain.setAutoVelocity(left, right);
		// System.out.println("Left Vel: " + (driveSignal.getLeft()) + "\nRight Vel: " +
		// (driveSignal.getRight()));
		// System.out.println("Left ticks Vel: " + (left) + "\nRight Vel: " + (right));
	}

	@Override
	public void teleopInit() 
	{
		BallIntake.ballIntakeinitialization();
		Arm.armInitialization();
		Elevator.elevatorInitialization();
		SeriesStateMachine.seriesStateMachineInit();
		// ShoppingCart.shoppingCartInit();
		Drivetrain.drivetrainInitialization();
	}

	@Override
	public void teleopPeriodic() 
	{
		if (mainController.leftBumper || Elevator.elevatorEncoderValue > 27000) 
		{
			Drivetrain.customArcadeDrive(mainController.rightJoyStickX * .65, mainController.leftJoyStickY * .6, gyro);
		} 
		else if (Arm.armEncoderValue > Constants.armSRXVerticalStowed && mainController.rightBumper) 
		{
      System.out.println("VISION MODE FOR BOTTOM LIMELIGHT");
			double joyY = mainController.leftJoyStickY;
			AutonomousSequences.limelightBottom.visionTargetingMode();
      AutonomousSequences.limelightBottom.center(0.1);
      AutonomousSequences.limelightTop.driverFlipped();
			double leftIn = AutonomousSequences.limelightBottom.leftSpeed + joyY;
			double rightIn = AutonomousSequences.limelightBottom.rightSpeed + joyY;
			Drivetrain.setPercentOutput(leftIn, rightIn);
		} 
		else if (Arm.armEncoderValue < Constants.armSRXVerticalStowed && mainController.rightBumper) 
		{
      System.out.println("VISION MODE FOR TOP LIMELIGHT");
			double joyY = mainController.leftJoyStickY;
			AutonomousSequences.limelightTop.visionTargetingMode();
      AutonomousSequences.limelightTop.center(0.1);
      AutonomousSequences.limelightBottom.driverFlipped();
			double leftIn = AutonomousSequences.limelightTop.leftSpeed + joyY;
			double rightIn = AutonomousSequences.limelightTop.rightSpeed + joyY;
			Drivetrain.setPercentOutput(leftIn, rightIn);
    }
    // else if(Arm.armEncoderValue > Constants.armSRXVerticalStowed)
    // {
    //   System.out.println("DRIVER MODE FOR BOTTOM LIMELIGHT. TOP FLIP");
    //   AutonomousSequences.limelightBottom.driverMode();
    //   AutonomousSequences.limelightTop.driverFlipped();
    //   Drivetrain.customArcadeDrive(mainController.rightJoyStickX * .7, mainController.leftJoyStickY, gyro);
    // }
    // else if(Arm.armEncoderValue < Constants.armSRXVerticalStowed)
    // {
    //   System.out.println("DRIVER MODE FOR TOP LIMELIGHT. BOTTOM FLIP");
    //   AutonomousSequences.limelightTop.driverMode();
    //   AutonomousSequences.limelightBottom.driverFlipped();
    //   Drivetrain.customArcadeDrive(mainController.rightJoyStickX * .7, mainController.leftJoyStickY, gyro);
    // }
		else 
		{
      System.out.println("BORK");
			AutonomousSequences.limelightBottom.driverMode();
			AutonomousSequences.limelightTop.driverMode();
			Drivetrain.customArcadeDrive(mainController.rightJoyStickX * .7, mainController.leftJoyStickY, gyro);
		}

		Arm.runArm();
		Elevator.runElevator();
		HatchGrabber.runHatchGrabber(coController.rightBumper);
		// ShoppingCart.runShoppingCart();
		SeriesStateMachine.setControllers(mainController, coController);
		SeriesStateMachine.runSeriesStateMachine();
		Elevator.printBannerSensor();
		Elevator.printElevatorEncoders();
		System.out.println("ELEVATOR AIMED STATE: " + Elevator.aimedState);
	}

	@Override
	public void disabledInit() 
	{
		AutonomousSequences.limelightTop.disabledMode();
		AutonomousSequences.limelightBottom.disabledMode();
		// // TestFunctions.vController.disabledMode();
		// Arm.disableArm();
		// Drivetrain.setToCoast();
		// Odometry.getInstance().closeOdoThread();
		// Elevator.aimedState = null;
		// SeriesStateMachine.aimedRobotState = null;
	}

	@Override
	public void disabledPeriodic() 
	{
		// Arm.armNEO.setIdleMode(IdleMode.kBrake);
		// Drivetrain.setToCoast();
		AutonomousSequences.limelightTop.disabledMode();
		AutonomousSequences.limelightBottom.disabledMode();
	}

	// private Timer secTimer;

	@Override
	public void testInit() 
	{
		// // Elevator.aimedState = ElevatorLevel.MINROTATE;
		Drivetrain.drivetrainInitialization();
		// // Drivetrain.initializeVelAccel();

		// // secTimer = new Timer();
		// // secTimer.reset();
		// // secTimer.start();
		// // TestFunctions.shuffleboard();
		// //BallIntake.ballIntakeinitialization();
		// // BallShooter.ballShooterinitialization();
		// // HatchIntake.hatchIntakeInitialization();
		// // BallShooter.ballShooterinitialization();
		// // Arm.aimedState = ArmPosition.REVLIMITSWITCH;
		// Elevator.elevatorInitialization();
		// Elevator.elevatorMaster.enableCurrentLimit(true);
		// Elevator.elevatorMaster.configContinuousCurrentLimit(50);
		// Arm.armInitialization()s;
	}

	@Override
	public void testPeriodic() 
	{
		// // Elevator.setOpenLoop(mainController.leftJoyStickY);
		// // // System.out.println("Controller power: " +
		// mainController.leftJoyStickY);
		// // // System.out.println("Elevator power: " +
		// Elevator.elevatorMaster.getMotorOutputPercent());
		// // // System.out.println("Elevator voltage: " +
		// Elevator.elevatorMaster.getMotorOutputVoltage());
		// // // System.out.println("Elevator currnet: " +
		// Elevator.elevatorMaster.getOutputCurrent());
		// Elevator.elevatorMaster.enableCurrentLimit(true);
		Drivetrain.customArcadeDrive(mainController.rightJoyStickX , mainController.leftJoyStickY, gyro);
		// ShoppingCart.runShoppingCartSPX(mainController.leftJoyStickY);

		// Elevator.setOpenLoop(mainController.rightJoyStickY);

	}

	public void updateJoysticks()
	{
		mainController.setMainContollerValues();
		coController.setMainContollerValues();
	}
}