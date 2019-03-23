package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3647autonomous.AutonomousSequences;
import frc.team3647autonomous.Odometry;
import frc.team3647inputs.*;
import frc.team3647subsystems.*;
import frc.team3647subsystems.Arm.ArmPosition;

public class Robot extends TimedRobot 
{
	public static PowerDistributionPanel pDistributionPanel;
	public static Joysticks mainController; 
	public static Joysticks coController;
	public static Gyro gyro;
  
	public static Notifier autoNotifier, teleopNotifier;

	public static double[] PIDFleft = new double[4]; 
	public static double[] PIDFright = new double[4]; 
	public static double[] PIDF = new double[4]; 
	public static int mVel = 0; 
	public static int mAccel = 0; 
	private Timer matchTimer;


	@Override
	public void robotInit()
	{
		pDistributionPanel = new PowerDistributionPanel();
		
		AutonomousSequences.limelightClimber.limelight.setToDriver();
		AutonomousSequences.limelightFourBar.limelight.setToDriver();
		// AutonomousSequences.limelightClimber.limelight.ledPipeline();
		// AutonomousSequences.limelightFourBar.limelight.ledPipeline();
		Shuffleboard.setRecordingFileNameFormat("San Diego Regional - ");
		matchTimer = new Timer();
		mainController = new Joysticks(0); 
		coController = new Joysticks(1); 
		gyro = new Gyro();
		
		Drivetrain.drivetrainInitialization();

		Arm.armNEO.setIdleMode(IdleMode.kBrake); 
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
		// Shuffleboard.startRecording();
		// matchTimer.reset();
		// matchTimer.start();

		try{gyro.resetAngle();}
		catch(NullPointerException e){ gyro = new Gyro(); }

		Drivetrain.drivetrainInitialization();
		Drivetrain.resetEncoders();
		AutonomousSequences.autoInitFWD("TestPath2");
		
		autoNotifier = new Notifier(() ->{
			Odometry.getInstance().runOdometry();
		});
		autoNotifier.startPeriodic(.01);
		AutonomousSequences.limelightClimber.limelight.setToRightContour();
		// Drivetrain.setEncoders(48951, 46817);
	}
  
	double left;
	double right;
  
	@Override
	public void autonomousPeriodic() 
	{
		AutonomousSequences.runPath();
	}

	@Override
	public void teleopInit()
	{
		Arm.armInitSensors();
		Drivetrain.drivetrainInitialization();
		Elevator.elevatorTeleopInit();
		AirCompressor.runCompressor();
		BallIntake.ballIntakeinitialization();
		Drivetrain.setToCoast();
	}

	//Teleop Code
	@Override
	public void teleopPeriodic()
	{
		driveVisionTeleop();
		Arm.runArm();
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
	}

	@Override
	public void disabledPeriodic()
	{
		// AutonomousSequences.limelightClimber.limelight.ledPipeline();
		// AutonomousSequences.limelightFourBar.limelight.ledPipeline();
	}

	// private Timer secTimer;

	//
	//l - 48951
	//r - 46817
	@Override
	public void testInit()
	{
		Drivetrain.drivetrainInitialization();
		Drivetrain.resetEncoders();
		Drivetrain.setToCoast();
	}


	@Override
	public void testPeriodic()
	{
		driveVisionTeleop();	
		// vision(VisionSide.kBackwards, (Elevator.elevatorEncoderValue > 27000), VisionContour.kRight);
		Drivetrain.printEncoders();	
	}

	public void updateJoysticks()
	{
		try{ mainController.setMainControllerValues(); }
		catch(NullPointerException e)
		{
			System.out.println(e);
			mainController = new Joysticks(0);
		}
		catch(Exception e){System.out.println(e);}
		
		try{ coController.setMainControllerValues();}
		catch(NullPointerException e)
		{
			System.out.println(e);
			coController = new Joysticks(1);
		}
		catch(Exception e){System.out.println(e);}
	}
	
	double threshold = Constants.limelightThreshold;
	public void driveVisionTeleop()
	{
		if(mainController.leftBumper)
			vision(getVisionSide(), (Elevator.elevatorEncoderValue > 27000), VisionContour.kLeft);
		else if(mainController.rightBumper)
			vision(getVisionSide(), (Elevator.elevatorEncoderValue > 27000), VisionContour.kRight);
		else 
		{
			AutonomousSequences.limelightClimber.driverMode();
			AutonomousSequences.limelightFourBar.driverMode();
			if(Elevator.elevatorEncoderValue > 27000)
				Drivetrain.customArcadeDrive(mainController.rightJoyStickX * .65, mainController.leftJoyStickY * .6);
			else
				Drivetrain.customArcadeDrive(mainController.rightJoyStickX * .7, mainController.leftJoyStickY);
		}
	}

	private VisionSide getVisionSide()
	{
		if(Arm.aimedState != null)
		{
			if(Arm.armEncoderValue > Constants.armSRXVerticalStowed || 
				Arm.aimedState.equals(ArmPosition.FLATBACKWARDS) 	|| 
				Arm.aimedState.equals(ArmPosition.CARGOL3FRONT)
				)
				return VisionSide.kBackwards;

			return VisionSide.kForwards;
		}
		else
		{
			if(Arm.armEncoderValue > Constants.armSRXVerticalStowed)
				return VisionSide.kBackwards;
			return VisionSide.kForwards;

		}
			
	}
	
	public enum VisionSide
	{
		kBackwards,
		kForwards,
		kUnknown,
	}
	public enum VisionContour
	{
		kRight,
		kLeft,
	}

	public void vision(VisionSide side, boolean scaleJoy, VisionContour contour)
	{
		double joyY = mainController.leftJoyStickY;
		double joyX = mainController.rightJoyStickX;
		VisionController mVision;

		if(scaleJoy)
		{
			joyY *= .6;
			joyX *= .65;
		}

		
		if(side.equals(VisionSide.kForwards))
			mVision = AutonomousSequences.limelightClimber;
		else
			mVision = AutonomousSequences.limelightFourBar;

		if(contour.equals(VisionContour.kRight))
			mVision.rightMost();
		else
			mVision.leftMost();
		
		if(mVision.area < 6)
		{
			mVision.center(Constants.limelightThreshold);
			Drivetrain.setPercentOutput(mVision.leftSpeed + joyY, mVision.rightSpeed + joyY);
		}
		else
			Drivetrain.customArcadeDrive(joyX * .7, joyY);
	}
}