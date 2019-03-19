package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3647autonomous.AutonomousSequences;
import frc.team3647autonomous.DriveSignal;
import frc.team3647autonomous.MotionProfileDirection;
import frc.team3647autonomous.Odometry;
import frc.team3647autonomous.RamseteFollower;
import frc.team3647autonomous.TrajectoryUtil;
import frc.team3647inputs.*;
import frc.team3647subsystems.*;
import frc.team3647subsystems.ShoppingCart.ShoppingCartPosition;
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
	public static Guitar guitarController;
	public static Gyro gyro;
  
	public static Notifier autoNotifier, teleopNotifier;

	public static double[] PIDFleft = new double[4]; 
	public static double[] PIDFright = new double[4]; 
	public static double[] PIDF = new double[4]; 
	public static int mVel = 0; 
	public static int mAccel = 0; 
	private Timer autoTimer;
	private Timer matchTimer;


	@Override
	public void robotInit()
	{
		Shuffleboard.setRecordingFileNameFormat("San Diego Regional - ");
		matchTimer = new Timer();
		mainController = new Joysticks(0); 
		coController = new Joysticks(1); 
		gyro = new Gyro();
		
		// TestFunctions.shuffleboardInit();
		// Drivetrain.drivetrainInitialization();
		// Drivetrain.initializeSmartDashboardVelAccel(); 

		Arm.armNEO.setIdleMode(IdleMode.kBrake); 
		autoTimer = new Timer();
		// CameraServer.getInstance().startAutomaticCapture(0); //USB Cam One
		// CameraServer.getInstance().startAutomaticCapture(1); //USB Cam Two
		
		matchTimer.reset();
	}

	@Override
	public void robotPeriodic()	
	{
		gyro.updateGyro();
		updateJoysticks();
		SmartDashboard.putNumber("Match Timer", 150 - matchTimer.get());
	}
	
  
	@Override
	public void autonomousInit() 
	{
		gyro.resetAngle(); 
		// The NEO takes the Motor-output in percent from the SRX and since SRX
		// values are using motion-magic, it "follows" the SRX
		Drivetrain.drivetrainInitialization();
		AutonomousSequences.autoInitFWD("YTW");
		// autoTimer.reset(); 
		// autoTimer.start(); 
		

		// Shuffleboard.startRecording();
		// matchTimer.reset();
		// matchTimer.start();

		
		Drivetrain.resetEncoders(); 
		// AirCompressor.runCompressor();

		// BallIntake.ballIntakeinitialization(); 
		// Arm.armInitialization(); 
		// Elevator.elevatorInitialization(); 
		// SeriesStateMachine.seriesStateMachineInit(); 
		// // ShoppingCart.shoppingCartInit(); 
		// Drivetrain.drivetrainInitialization(); 

		autoNotifier = new Notifier(() ->
		{
			// Arm.armNEOFollow();
			Odometry.getInstance().runOdometry();
		});
		autoNotifier.startPeriodic(0.01);

		// teleopNotifier = new Notifier(() -> 
    	// {
		// 	Arm.armNEOFollow(); 
		// }); 
		// teleopNotifier.startPeriodic(.01);
		Drivetrain.selectPIDF(Constants.velocitySlotIdx, Constants.rightVelocityPIDF, Constants.leftVelocityPIDF);

	}
  
	double left;
	double right;
  
	@Override
	public void autonomousPeriodic() 
	{
		// Drivetrain.selectPIDF(Constants.velocitySlotIdx, Constants.rightVelocityPIDF, Constants.leftVelocityPIDF);
		// AutonomousSequences.fwdBwd("RocketToStation");
		AutonomousSequences.runPath();
		System.out.println(Math.toRadians(gyro.getYaw()));
		System.out.println("Current Position: " + Odometry.getInstance().getCurrentEncoderPosition());
		// Drivetrain.printVelocity();
		// System.out.println("Desired Left Vel: " + AutonomousSequences.leftSpeed);
		// System.out.println("Desired Right Vel: " + AutonomousSequences.rightSpeed);


		
		// teleopPeriodic();
	}

	@Override
	public void teleopInit()
	{
		//RUN NOTHING
	}

	//Teleop Code
	@Override
	public void teleopPeriodic()
	{
		driveVisionTeleop();
		Arm.runArm();
		// if(SeriesStateMachine.climbMode)
		// {
		// 	ShoppingCart.runShoppingCartSPX(mainController.leftJoyStickY);
		// 	if(mainController.buttonB)
		// 		ShoppingCart.aimedState = ShoppingCartPosition.MIDDLE;
		// }
		
		// ShoppingCart.runShoppingCart();
		Elevator.runElevator(); 
		HatchGrabber.runHatchGrabber(coController);
		SeriesStateMachine.setControllers(mainController, coController); 
		SeriesStateMachine.runSeriesStateMachine();
	}

	@Override
	public void disabledInit()
	{
		Shuffleboard.stopRecording();
		matchTimer.stop();
		matchTimer.reset();
		// Arm.disableArm();
		// Drivetrain.setToCoast();
		// Odometry.getInstance().closeOdoThread();
		// Elevator.aimedState = null;
		// SeriesStateMachine.aimedRobotState = null;

	}

	@Override
	public void disabledPeriodic()
	{
		AutonomousSequences.limelightClimber.rightMost();
		AutonomousSequences.limelightFourBar.rightMost();
	}

	// private Timer secTimer;

	@Override
	public void testInit()
	{
		HatchGrabber.stopMotor();
		// ShoppingCart.shoppingCartInit();
		// Drivetrain.drivetrainInitialization();
		// Elevator.elevatorInitialization();
		Drivetrain.drivetrainInitialization();
		Drivetrain.resetEncoders();
		Drivetrain.selectPIDF(Constants.velocitySlotIdx, Constants.rightVelocityPIDF, Constants.leftVelocityPIDF);
	}


	@Override
	public void testPeriodic()
	{
		// System.out.println(gyro.getYaw());
		if(mainController.buttonA)
		{
			// Drivetrain.setVelocity(mainController.rightJoyStickX, -mainController.rightJoyStickX);
			gyro.resetAngle();
			Drivetrain.resetEncoders();
		}
		// else
		// 	Drivetrain.setVelocity(mainController.leftJoyStickY, mainController.leftJoyStickY);

		// Drivetrain.printVelError();

		// if(mainController.buttonY)
		// {
		// 	Drivetrain.leftSRX.set(ControlMode.Velocity, 3000);
		// 	Drivetrain.rightSRX.set(ControlMode.Velocity, 3000);
		// }
		// else if(mainController.buttonA)
		// {
		// 	Drivetrain.leftSRX.set(ControlMode.Velocity, -600);
		// 	Drivetrain.rightSRX.set(ControlMode.Velocity, -600);
		// }
		// else
		// {
		// 	Drivetrain.stop();
		// }

		// Drivetrain.printVelocity();

		// HatchGrabber.runHatchGrabber(coController);
		// Drivetrain.printEncoders();
		// gyro.printAngles();
		// if(Drivetrain.rightSRX.getSelectedSensorPosition(0) != 0)
			// System.out.println(Drivetrain.leftSRX.getSelectedSensorPosition(0) / Drivetrain.rightSRX.getSelectedSensorPosition(0));
		// Drivetrain.printAccel();
		
		teleopVisionForward(AutonomousSequences.limelightClimber, AutonomousSequences.limelightFourBar, threshold);
		// Drivetrain.customArcadeDrive(mainController.rightJoyStickX, mainController.leftJoyStickY, gyro);
	}

	public void updateJoysticks()
	{
		mainController.setMainControllerValues();
		coController.setMainControllerValues();
	}
	
	double threshold = Constants.limelightThreshold;
	public void driveVisionTeleop()
	{
		if (Elevator.elevatorEncoderValue > 27000) 
		{
			Drivetrain.customArcadeDrive(mainController.rightJoyStickX * .65, mainController.leftJoyStickY * .6, gyro);
		} 
		else if (Arm.armEncoderValue > Constants.armSRXVerticalStowed) //Hatch intake above fourbar
		{
			SmartDashboard.putString("ARM ORIENTATION", "HATCH BACKWARDS");
			teleopVisionBackward(AutonomousSequences.limelightFourBar, AutonomousSequences.limelightClimber, threshold);

		} 
		else if (Arm.armEncoderValue < Constants.armSRXVerticalStowed)//Hatch intake above climber
 		{
			SmartDashboard.putString("ARM ORIENTATION", "HATCH FORWARDS");
			teleopVisionForward(AutonomousSequences.limelightClimber, AutonomousSequences.limelightFourBar, threshold);
		} 
		else 
		{
			AutonomousSequences.limelightClimber.driverMode();
			AutonomousSequences.limelightFourBar.driverMode();
			Drivetrain.customArcadeDrive(mainController.rightJoyStickX * .7, mainController.leftJoyStickY, gyro);
		}


	}
	
	
	int visionCase = 0;
	public void teleopVisionForward(VisionController camOne, VisionController camTwo, double threshold) //.0037
	{
		double joyY = mainController.leftJoyStickY;
		camTwo.driverFlipped();
		if (mainController.rightBumper) 
		{
			camOne.rightMost();
			switch (visionCase) 
			{
			case 0:
				camOne.center(threshold);
				Drivetrain.setPercentOutput(camOne.leftSpeed + joyY, camOne.rightSpeed + joyY);
				if (camOne.area > 6)
					visionCase = 1;
				break;

			case 1:
				Drivetrain.customArcadeDrive(mainController.rightJoyStickX * .7, mainController.leftJoyStickY, gyro);

			}
		} 
		else if (mainController.leftBumper) 
		{
			camOne.leftMost();
			switch (visionCase) {
			case 0:
				camOne.center(threshold);
				Drivetrain.setPercentOutput(camOne.leftSpeed + joyY, camOne.rightSpeed + joyY);
				if (camOne.area > 6)
					visionCase = 1;
				break;

			case 1:
				Drivetrain.customArcadeDrive(mainController.rightJoyStickX * .7, mainController.leftJoyStickY, gyro);

			}
		} 
		else 
		{
			visionCase = 0;
			Drivetrain.customArcadeDrive(mainController.rightJoyStickX * .7, mainController.leftJoyStickY, gyro);
		}
	}
	
	public void teleopVisionBackward(VisionController camOne, VisionController camTwo, double threshold) //.0037
	{
		double joyY = mainController.leftJoyStickY;
		camTwo.driverFlipped();
		if (mainController.rightBumper) 
		{
			camOne.leftMost();
			switch(visionCase)
			{
			case 0:
				camOne.center(threshold);
				Drivetrain.setPercentOutput(-camOne.rightSpeed + joyY, -camOne.leftSpeed + joyY);
				if(camOne.area > 6)
						visionCase = 1;
					break;
				
			case 1:
				Drivetrain.customArcadeDrive(mainController.rightJoyStickX * .7, mainController.leftJoyStickY, gyro);
						
			}
		} 
		else if (mainController.leftBumper) 
		{
			camOne.rightMost();
			switch(visionCase)
			{
			case 0:
				camOne.center(threshold);
				Drivetrain.setPercentOutput(-camOne.rightSpeed + joyY, -camOne.leftSpeed + joyY);
				if(camOne.area > 6)
						visionCase = 1;
					break;
				
			case 1:
				Drivetrain.customArcadeDrive(mainController.rightJoyStickX * .7, mainController.leftJoyStickY, gyro);
						
			}
		} 
		else 
		{
			visionCase = 0;
			Drivetrain.customArcadeDrive(mainController.rightJoyStickX * .7, mainController.leftJoyStickY, gyro);
		}
	}
}