package frc.robot;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3647autonomous.AutonomousSequences;
import frc.team3647inputs.*;
import frc.team3647subsystems.*;
import frc.team3647subsystems.Arm.ArmPosition;
import frc.team3647subsystems.Elevator.ElevatorLevel;

public class Robot extends TimedRobot 
{
	
	public static PowerDistributionPanel pDistributionPanel;
	public static Joysticks mainController; 
	public static Joysticks coController;
	public static Gyro gyro;
  
	public static Notifier autoNotifier, drivetrainNotifier, armFollowerNotifier, stateMachineRunnerNotifier;

	

	public static double[] PIDFleft = new double[4]; 
	public static double[] PIDFright = new double[4]; 
	public static double[] PIDF = new double[4]; 
	public static int mVel = 0; 
	public static int mAccel = 0; 
	private Timer matchTimer;

	public static boolean cargoDetection;

	public Robot()
	{
		super(.02);
	}

	@Override
	public void robotInit()
	{
		pDistributionPanel = new PowerDistributionPanel();
		
		AutonomousSequences.limelightClimber.limelight.setToBlack();
		AutonomousSequences.limelightFourBar.limelight.setToBlack();
		matchTimer = new Timer();
		mainController = new Joysticks(0); 
		coController = new Joysticks(1); 
		gyro = new Gyro();
		
		Drivetrain.init();

		Arm.setToBrake();
		// CameraServer.getInstance().startAutomaticCapture().setVideoMode(PixelFormat.kMJPEG, 160, 120, 15); //USB Cam One
		// CameraServer.getInstance().startAutomaticCapture(); //USB Cam One
		// CameraServer.getInstance().startAutomaticCapture().close();
		
		matchTimer.reset();

		armFollowerNotifier = new Notifier(() -> {
			Arm.armNEOFollow();
		});

		drivetrainNotifier = new Notifier(() ->{
			updateJoysticks();
			driveVisionTeleop();
		});

		stateMachineRunnerNotifier = new Notifier(() ->{
			Arm.updateEncoder();
			Elevator.updateEncoder();
			SeriesStateMachine.setControllers(mainController, coController);
			SeriesStateMachine.run();
		});


		
		
	}

	@Override
	public void robotPeriodic()	
	{
		gyro.update();
		// updateJoysticks(); Done in drivetrain notifier!
		SmartDashboard.putNumber("Match Timer", 150 - matchTimer.get());
		cargoDetection = BallShooter.cargoDetection();
	}
	
  
	Timer stateMachineTimer = new Timer();
	boolean runStateMachine = false;
	@Override
	public void autonomousInit() 
	{

		try{gyro.resetAngle();}
		catch(NullPointerException e){ gyro = new Gyro(); }

		Drivetrain.init();
		Arm.init();
		Elevator.init();
		SeriesStateMachine.init();
		AirCompressor.run();
		BallIntake.init();
		Drivetrain.resetEncoders();

		Drivetrain.setToCoast();

		drivetrainNotifier.startPeriodic(.02);
		armFollowerNotifier.startPeriodic(.01);
		stateMachineRunnerNotifier.startPeriodic(.02);
	}
  
	@Override
	public void autonomousPeriodic() 
	{
		teleopPeriodic();
	}

	@Override
	public void teleopInit()
	{
		Drivetrain.init();
		Arm.initSensors();
		Elevator.initSensors();

		AirCompressor.run();
		BallIntake.init();

		Drivetrain.setToCoast();

		drivetrainNotifier.startPeriodic(.02);
		armFollowerNotifier.startPeriodic(.01);
		stateMachineRunnerNotifier.startPeriodic(.02);
	}

	//Teleop Code
	@Override
	public void teleopPeriodic()
	{
		Arm.run();
		Elevator.run(); 
		HatchGrabber.run(coController);
		//State machine will run with notifier
		//Drivetrain runs with notifier
	}

	@Override
	public void disabledInit()
	{
		drivetrainNotifier.stop();
		armFollowerNotifier.stop();
		stateMachineRunnerNotifier.stop();

		Shuffleboard.stopRecording();
		matchTimer.stop();
		matchTimer.reset();
	}

	@Override
	public void disabledPeriodic()
	{
		AutonomousSequences.limelightClimber.limelight.setToBlack();
		AutonomousSequences.limelightFourBar.limelight.setToBlack();
	}

	@Override
	public void testInit()
	{
		// Drivetrain.init();
		// Drivetrain.resetEncoders();
		// Drivetrain.setToCoast();
		// drivetrainNotifier.startPeriodic(.02);
		
	}
	@Override
	public void testPeriodic()
	{
		// Drivetrain.setPercentOutput(-.5, .5);
		
	}

	private void updateJoysticks()
	{
		try{ mainController.update(); }
		catch(NullPointerException e)
		{
			System.out.println(e);
			mainController = new Joysticks(0);
		}
		catch(Exception e){System.out.println(e);}
		
		try{ coController.update();}
		catch(NullPointerException e)
		{
			System.out.println(e);
			coController = new Joysticks(1);
		}
		catch(Exception e){System.out.println(e);}
	}
	
	double threshold = Constants.limelightThreshold;
	private void driveVisionTeleop()
	{
		if(mainController.leftBumper)
			vision(getVisionSide(), (Elevator.encoderValue > 27000), VisionContour.kLeft);
		else if(mainController.rightBumper)
			vision(getVisionSide(), (Elevator.encoderValue > 27000), VisionContour.kRight);
		else 
		{
			AutonomousSequences.limelightClimber.disabledMode();
			AutonomousSequences.limelightFourBar.disabledMode();
			AutonomousSequences.limelightArmBottom.disabledMode();
			AutonomousSequences.limelightArmTop.disabledMode();
			if(Elevator.encoderValue > 27000)
				Drivetrain.customArcadeDrive(mainController.rightJoyStickX * .65, mainController.leftJoyStickY * .6);
			else
				Drivetrain.customArcadeDrive(mainController.rightJoyStickX * .7, mainController.leftJoyStickY);
		}
	}

	private VisionSide getVisionSide()
	{
		boolean armAfterVerticalStowed = Arm.encoderValue > Constants.armSRXVerticalStowed;
		if(Arm.aimedState != null)
		{
			if(Arm.aimedState.equals(ArmPosition.FLATBACKWARDS) || 
				armAfterVerticalStowed || 
				Arm.aimedState.equals(ArmPosition.CARGOL3FRONT)
			  )
				return VisionSide.kBackwards;
			
			else if(Arm.aimedState.equals(ArmPosition.FLATFORWARDS) || 
					!armAfterVerticalStowed || 
					Arm.aimedState.equals(ArmPosition.CARGOL3BACK)
		  		   )
				return VisionSide.kForwards;


			return VisionSide.kForwards;
		}
		else
		{
			if(armAfterVerticalStowed)
				return VisionSide.kBackwards;
			return VisionSide.kForwards;
		}
			
	}
	
	private enum VisionSide
	{
		kBackwards,
		kForwards,
		kTop,
		kBottom,
		kUnknown,
	}
	private enum VisionContour
	{
		kRight,
		kLeft,
		kClosest
	}

	private void vision(VisionSide side, boolean scaleJoy, VisionContour contour)
	{
		double joyY = mainController.leftJoyStickY;
		double joyX = mainController.rightJoyStickX;
		VisionController mVision = getLimelight(side);

		if(scaleJoy)
		{
			joyY *= .6;
			joyX *= .65;
		}

		if(contour.equals(VisionContour.kRight))
			mVision.closestMode();
		else if (contour.equals(VisionContour.kLeft))
			mVision.closestMode();
		else
			mVision.closestMode();
		
		if(mVision.area < 6)
		{
			mVision.center(Constants.limelightThreshold);
			Drivetrain.setPercentOutput(mVision.leftSpeed + joyY, mVision.rightSpeed + joyY);
		}
		else
			Drivetrain.customArcadeDrive(joyX * .7, joyY);
	}

	private VisionController getLimelight(VisionSide side)
	{
		boolean elevatorAimedState = Elevator.aimedState.equals(ElevatorLevel.BOTTOM);

		if(side.equals(VisionSide.kForwards))
		{
			if(elevatorAimedState)
				return AutonomousSequences.limelightArmTop;
			else if(cargoDetection)
				return AutonomousSequences.limelightFourBar;
			else
				return AutonomousSequences.limelightClimber;
		}
		else
		{
			if(elevatorAimedState)
				return AutonomousSequences.limelightArmBottom;
			else if(cargoDetection)
				return AutonomousSequences.limelightClimber;
			else
				return AutonomousSequences.limelightFourBar;
		}	
	}
}