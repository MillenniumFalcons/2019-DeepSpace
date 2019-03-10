package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3647autonomous.AutonomousSequences; 
import frc.team3647autonomous.Odometry; 
import frc.team3647inputs. * ; 
import frc.team3647subsystems. * ;
import frc.team3647subsystems.ShoppingCart.ShoppingCartPosition; 

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
	private Timer matchTimer;


	@Override
	public void robotInit()
	{
		AutonomousSequences.limelightClimber.limelight.setToDriver();
		AutonomousSequences.limelightFourBar.limelight.setToDriver();
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
		// CameraServer.getInstance().startAutomaticCapture().setVideoMode(PixelFormat.kMJPEG, 160, 120, 15); //USB Cam One
		// CameraServer.getInstance().startAutomaticCapture(); //USB Cam One
		// CameraServer.getInstance().startAutomaticCapture().close();
		
		matchTimer.reset();

		teleopNotifier = new Notifier(() -> {
			Arm.armNEOFollow();
		});
		teleopNotifier.startPeriodic(.01);
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
		// The NEO takes the Motor-output in percent from the SRX and since SRX
		// values are using motion-magic, it "follows" the SRX
		// AutonomousSequences.autoInit("Level2RightToCargoRight"); 
		// autoTimer.reset(); 
		// autoTimer.start(); 
		// autoNotifier = new Notifier(() ->
		// {
		// 	Arm.armNEOFollow();
		// 	Odometry.getInstance().runOdometry();
		// });
		// autoNotifier.startPeriodic(0.01);

		Shuffleboard.startRecording();
		matchTimer.reset();
		matchTimer.start();

		gyro.resetAngle(); 
		Drivetrain.drivetrainInitialization(); 
		Drivetrain.resetEncoders(); 
		AirCompressor.runCompressor(); 

		BallIntake.ballIntakeinitialization(); 
		Arm.armInitialization(); 
		Elevator.elevatorInitialization(); 
		SeriesStateMachine.seriesStateMachineInit(); 
		// ShoppingCart.shoppingCartInit(); 
		Drivetrain.drivetrainInitialization(); 

		// teleopNotifier = new Notifier(() -> 
    	// {
		// 	Arm.armNEOFollow(); 
		// }); 
		// teleopNotifier.startPeriodic(.01);
	}


	@Override
	public void autonomousPeriodic()
	{
		teleopPeriodic();
	}

	@Override
	public void teleopInit()
	{
		//RUN NOTHING

		Arm.armInitSensors();
		SeriesStateMachine.initializeTeleop();
		Drivetrain.drivetrainInitialization();
		Elevator.elevatorTeleopInit();
		AirCompressor.runCompressor();
		BallIntake.ballIntakeinitialization();

		

	}

	//Teleop Code
	@Override
	public void teleopPeriodic()
	{
		if(matchTimer.get() == 148)
		{
			Shuffleboard.stopRecording();
			matchTimer.stop();
		}
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
		// Arm.disableArm();
		// Drivetrain.setToCoast();
		// Odometry.getInstance().closeOdoThread();
		// Elevator.aimedState = null;
		// SeriesStateMachine.aimedRobotState = null;
	}

	@Override
	public void disabledPeriodic()
	{
		// AutonomousSequences.limelightClimber.rightMost();
		// AutonomousSequences.limelightFourBar.rightMost();

		// Arm.armNEO.setIdleMode(IdleMode.kBrake);
		// Drivetrain.setToCoast();
		// AutonomousSequences.limelightTop.disabledMode(); 
		// AutonomousSequences.limelightBottom.disabledMode(); 
	}

	// private Timer secTimer;

	@Override
	public void testInit()
	{
		// ShoppingCart.shoppingCartInit();
		// Drivetrain.drivetrainInitialization();
		// Elevator.elevatorInitialization();
		// Drivetrain.drivetrainInitialization();
		// Arm.armInitialization();
	}


	@Override
	public void testPeriodic()
	{
		// HatchGrabber.runHatchGrabber(coController);
		// driveVisionTeleop();
		// try
		// {
		// 	System.out.println("Arm fwd limit switch : " + Arm.getFwdLimitSwitch());
		// }	
		// catch(Exception e)
		// {
		// 	System.out.println(e.toString());
		// }
			HatchGrabber.runConstant();
	}

	public void updateJoysticks()
	{
		mainController.setMainContollerValues();
		coController.setMainContollerValues();
	}
	
	public void driveVisionTeleop()
	{
		if (Elevator.elevatorEncoderValue > 27000) 
		{
			Drivetrain.customArcadeDrive(mainController.rightJoyStickX * .65, mainController.leftJoyStickY * .6, gyro);
			if (Arm.armEncoderValue > Constants.armSRXVerticalStowed && !BallShooter.cargoDetection()) //Hatch intake above fourbar
			{
				SmartDashboard.putString("ARM ORIENTATION", "HATCH BACKWARDS");
				Drivetrain.customArcadeDrive(mainController.rightJoyStickX * .65, mainController.leftJoyStickY * .6, gyro);

				// teleopVisionBackward(AutonomousSequences.limelightFourBar, AutonomousSequences.limelightClimber, .075);

			} 
			else if (Arm.armEncoderValue < Constants.armSRXVerticalStowed && !BallShooter.cargoDetection())//Hatch intake above climber
			{
				Drivetrain.customArcadeDrive(mainController.rightJoyStickX * .65, mainController.leftJoyStickY * .6, gyro);
				SmartDashboard.putString("ARM ORIENTATION", "HATCH FORWARDS");
				// teleopVisionForward(AutonomousSequences.limelightClimber, AutonomousSequences.limelightFourBar, .075);
			}
			else if(BallShooter.cargoDetection())
			{
				// AutonomousSequences.limelightClimber.driverMode();
				// AutonomousSequences.limelightFourBar.driverMode();
				Drivetrain.customArcadeDrive(mainController.rightJoyStickX * .7, mainController.leftJoyStickY, gyro);
			}
			else
			{
				// AutonomousSequences.limelightClimber.driverMode();
				// AutonomousSequences.limelightFourBar.driverMode();
				Drivetrain.customArcadeDrive(mainController.rightJoyStickX * .7, mainController.leftJoyStickY, gyro);
			}
		} 
		else if(BallShooter.cargoDetection())
		{
			// AutonomousSequences.limelightClimber.driverMode();
			// AutonomousSequences.limelightFourBar.driverMode();
			Drivetrain.customArcadeDrive(mainController.rightJoyStickX * .7, mainController.leftJoyStickY, gyro);
		}
		else if (Arm.armEncoderValue > Constants.armSRXVerticalStowed && !BallShooter.cargoDetection()) //Hatch intake above fourbar
		{
			Drivetrain.customArcadeDrive(mainController.rightJoyStickX * .65, mainController.leftJoyStickY * .6, gyro);

			SmartDashboard.putString("ARM ORIENTATION", "HATCH BACKWARDS");
			// teleopVisionBackward(AutonomousSequences.limelightFourBar, AutonomousSequences.limelightClimber, .075);

		} 
		else if (Arm.armEncoderValue < Constants.armSRXVerticalStowed && !BallShooter.cargoDetection())//Hatch intake above climber
		{
			Drivetrain.customArcadeDrive(mainController.rightJoyStickX * .65, mainController.leftJoyStickY * .6, gyro);
			SmartDashboard.putString("ARM ORIENTATION", "HATCH FORWARDS");
			// teleopVisionForward(AutonomousSequences.limelightClimber, AutonomousSequences.limelightFourBar, .075);
		} 
		else 
		{
			// AutonomousSequences.limelightClimber.driverMode();
			// AutonomousSequences.limelightFourBar.driverMode();
			Drivetrain.customArcadeDrive(mainController.rightJoyStickX * .7, mainController.leftJoyStickY, gyro);
		}


	}
	
	double targetTA = 5.00;
	int visionCase = 0;
	public void teleopVisionForward(VisionController camOne, VisionController camTwo, double threshold) //.075
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
				if (camOne.area > targetTA)
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
				if (camOne.area > targetTA)
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
	
	public void teleopVisionBackward(VisionController camOne, VisionController camTwo, double threshold) //.075
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
				if(camOne.area > targetTA)
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
				if(camOne.area > targetTA)
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