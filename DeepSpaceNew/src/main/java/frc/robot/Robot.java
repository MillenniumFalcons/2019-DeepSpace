package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3647autonomous.AutonomousSequences;
import frc.team3647autonomous.Odometry;
import frc.team3647inputs.*;
import frc.team3647subsystems.*;

public class Robot extends TimedRobot 
{
	// private static final String kDefaultAuto = "Default";
	// private static final String kCustomAuto = "My Auto";
	// private String m_autoSelected;
	// private final SendableChooser<String> m_chooser = new SendableChooser<>();

	public static Joysticks mainController;
	public static Joysticks coController;
  public static Gyro gyro;
  
  public static Notifier autoNotifier, teleopNotifier;

	public static double[] PIDFleft = new double[4];
	public static double[] PIDFright = new double[4];
	public static double[] PIDF = new double[4];
	public static int mVel = 0;
	public static int mAccel = 0;
	private Timer autoTimer;

	@Override
	public void robotInit() 
	{

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
	

	@Override
	public void autonomousInit() 
	{
    	// The NEO takes the Motor-output in percent from the SRX and since SRX values are using motion-magic, it "follows" the SRX
		autoNotifier = new Notifier(()->
		{
			Arm.armNEOFollow();
      		Odometry.getInstance().runOdometry();
    	});
    	autoNotifier.startPeriodic(0.01);

		gyro.resetAngle();
		Drivetrain.drivetrainInitialization();
		Drivetrain.resetEncoders();
		AirCompressor.runCompressor();
		AutonomousSequences.autoInit("Level2RightToCargoRight");
		autoTimer.reset();
		autoTimer.start();

	}


	@Override
	public void autonomousPeriodic() 
	{
		if(autoTimer.get() > 2)
		{
			// SeriesStateMachine.runSeriesStateMachine();
			// Elevator.runElevator();
			// Arm.runArm();
		}
		AutonomousSequences.level2RightToCargoShipRight();
	}

	@Override
	public void teleopInit() 
	{
		BallIntake.ballIntakeinitialization();
		Arm.armInitialization();
		Elevator.elevatorInitialization();
		SeriesStateMachine.seriesStateMachineInit();
		ShoppingCart.shoppingCartInit();
		Drivetrain.drivetrainInitialization();

		teleopNotifier = new Notifier(() -> {
			Arm.armNEOFollow();
		});
		teleopNotifier.startPeriodic(.01);

	}

	//Teleop Code
	@Override
	public void teleopPeriodic() 
	{
		if (mainController.leftBumper || Elevator.elevatorEncoderValue > 27000) 
		{
			Drivetrain.customArcadeDrive(mainController.rightJoyStickX * .65, mainController.leftJoyStickY * .6, gyro);
		} 
		else if (Arm.armEncoderValue > Constants.armSRXVerticalStowed) 
		{
			SmartDashboard.putString("ARM ORIENTATION", "HATCH BACKWARDS");
			double joyY = mainController.leftJoyStickY;
			AutonomousSequences.limelightBottom.visionTargetingMode();
			AutonomousSequences.limelightTop.driverFlipped();
			if (mainController.rightBumper)
			{
				AutonomousSequences.limelightBottom.center(0.1);
				double leftIn = AutonomousSequences.limelightBottom.leftSpeed + joyY;
				double rightIn = AutonomousSequences.limelightBottom.rightSpeed + joyY;
				Drivetrain.setPercentOutput(leftIn, rightIn);
			}
			else 
			{
				Drivetrain.customArcadeDrive(mainController.rightJoyStickX * .7, mainController.leftJoyStickY, gyro);	
			}
		} 
		else if (Arm.armEncoderValue < Constants.armSRXVerticalStowed)
		{
			SmartDashboard.putString("ARM ORIENTATION", "HATCH FORWARDS");
			double joyY = mainController.leftJoyStickY;
			AutonomousSequences.limelightTop.visionTargetingMode();
			AutonomousSequences.limelightBottom.driverFlipped();
			if (mainController.rightBumper)
			{
				AutonomousSequences.limelightTop.center(0.1);
				double leftIn = AutonomousSequences.limelightTop.leftSpeed + joyY;
				double rightIn = AutonomousSequences.limelightTop.rightSpeed + joyY;
				Drivetrain.setPercentOutput(leftIn, rightIn);
			}
			else 
			{
				Drivetrain.customArcadeDrive(mainController.rightJoyStickX * .7, mainController.leftJoyStickY, gyro);	
			}
    	}
		else 
		{
			System.out.println("Shouldn't be here, but");
			AutonomousSequences.limelightBottom.driverMode();
			AutonomousSequences.limelightTop.driverMode();
			Drivetrain.customArcadeDrive(mainController.rightJoyStickX * .7, mainController.leftJoyStickY, gyro);
		}

		Arm.runArm();
		Arm.printArmEncoders();
		Elevator.runElevator();
		Elevator.printElevatorEncoders();
		HatchGrabber.runHatchGrabber(coController.rightBumper);
		ShoppingCart.runShoppingCart();
		if (SeriesStateMachine.climbMode)
		{
			ShoppingCart.runShoppingCartSPX(mainController.leftJoyStickY);
		}
		else if(!SeriesStateMachine.climbMode)
		{
			ShoppingCart.setPosition(0);
			ShoppingCart.runShoppingCartSPX(0);
		}
		SeriesStateMachine.setControllers(mainController, coController);
		SeriesStateMachine.runSeriesStateMachine();
	}

	@Override
	public void disabledInit() 
	{
		AutonomousSequences.limelightTop.disabledMode();
		AutonomousSequences.limelightBottom.disabledMode();
		// TestFunctions.vController.disabledMode();
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
		Elevator.elevatorInitialization();
		// Elevator.elevatorMaster.enableCurrentLimit(true);
		// Elevator.elevatorMaster.configContinuousCurrentLimit(50);
    	// Arm.armInitialization()s;
    
		// ShoppingCart.shoppingCartInit();
	}

	@Override
	public void testPeriodic() 
	{
		// Mop.retractMop();
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
		// Drivetrain.customArcadeDrive(mainController.rightJoyStickX , mainController.leftJoyStickY, gyro);
		// ShoppingCart.runShoppingCartSPX(mainController.leftJoyStickY);
    	// BallShooter.intakeMotor.set(ControlMode.PercentOutput, -.4);
		Elevator.setOpenLoop(mainController.rightJoyStickY);
		// ShoppingCart.setShoppinCartEncoder();
		// ShoppingCart.printPosition();
		// ShoppingCart.shoppingCartMotor.set(ControlMode.PercentOutput, mainController.leftJoyStic kY);
	}

	public void updateJoysticks()
	{
		mainController.setMainContollerValues();
		coController.setMainContollerValues();
	}
}