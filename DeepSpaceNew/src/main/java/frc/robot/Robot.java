package frc.robot;

// import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3647autonomous.AutonomousSequences;
import frc.team3647autonomous.Odometry;
// import frc.team3647autonomous.Odometry;
import frc.team3647inputs.*;
import frc.team3647subsystems.*;
import frc.team3647subsystems.SeriesStateMachine.ScoringPosition;
import frc.team3647subsystems.VisionController.VisionMode;
import frc.team3647utility.AutoChooser;
import frc.team3647utility.Path;

public class Robot extends TimedRobot 
{
	
	public static PowerDistributionPanel pDistributionPanel;
	public static Joysticks mainController; 
	public static Joysticks coController;
	public static Gyro gyro;
  
	public static Notifier drivetrainNotifier, armFollowerNotifier, autoNotifier, pathNotifier, subsystemsEncodersNotifier;

	public static boolean runAuto = true;

	public static boolean cargoDetection;

	private AutoChooser mAutoChooser;

	private Path mPath;

	@Override
	public void robotInit()
	{
		LiveWindow.disableAllTelemetry();
		LiveWindow.setEnabled(false);
		pDistributionPanel = new PowerDistributionPanel();
		
		AutonomousSequences.limelightClimber.limelight.set(VisionMode.kBlack);
		AutonomousSequences.limelightFourBar.limelight.set(VisionMode.kBlack);
		mainController = new Joysticks(0); 
		coController = new Joysticks(1); 
		gyro = new Gyro();
		
		Drivetrain.init();

		Arm.setToBrake();
		subsystemsEncodersNotifier = new Notifier(() -> {
			Elevator.updateEncoder();
			Arm.updateEncoder();
		});
		armFollowerNotifier = new Notifier(() -> {
			Arm.armNEOFollow();
		});

		drivetrainNotifier = new Notifier(() ->{
			mainController.update();
			driveVisionTeleop();
		});

		autoNotifier = new Notifier(() ->{
			Drivetrain.updateEncoders();
			Odometry.getInstance().runOdometry();
		});
		
		pathNotifier = new Notifier(() ->{
			// AutonomousSequences.frontRocketAuto("Right");
			// AutonomousSequences.sideCargoShipAuto();
			AutonomousSequences.mixedRocketAuto("Left");
		});

		mAutoChooser = new AutoChooser();	
		mAutoChooser.update();
		mPath = new Path(mAutoChooser.getSide(), mAutoChooser.getStruct(), mAutoChooser.getMode());
	}

	@Override
	public void robotPeriodic()	
	{
		if(isAutonomous())
			gyro.update();
		if(isEnabled())
			coController.update();
		else{
			mAutoChooser.update();
			mPath.update(mAutoChooser.getSide(), mAutoChooser.getStruct(), mAutoChooser.getMode());
			SmartDashboard.putString("Path running first", mPath.getIntialPath());
		}
		// mainController.update(); Done in drivetrain notifier!
		// SmartDashboard.putNumber("Match Timer", DriverStation.getInstance().getMatchTime());
		cargoDetection = BallShooter.cargoDetection();
	}
	
  
	@Override
	public void autonomousInit() 
	{
		try{gyro.resetAngle();}
		catch(NullPointerException e){ gyro = new Gyro(); }

		runAuto = true;
		AutonomousSequences.autoStep = 0;
		
		Drivetrain.init();
		Drivetrain.setToBrake();
		Drivetrain.resetEncoders();

		Arm.init();
		Elevator.init();
		SeriesStateMachine.init();
		BallIntake.init();
		MiniShoppingCart.init();
		

		
		
		
		pathNotifier = new Notifier(() ->{
			mPath.run();
		});
		AutonomousSequences.autoInitFWD(mPath.getIntialPath()); //off lvl 2

		// AutonomousSequences.autoInitFWD("LeftPlatformToBackLeftRocket"); //off lvl 1
		// AutonomousSequences.autoInitFWD("LeftPlatformToBackLeftRocket"); //mixed left rocket
		// AutonomousSequences.autoInitFWD("PlatformToLeftMiddleLeftCargoShip"); //cargoship left
		// AutonomousSequences.autoInitFWD("RightPlatformToRightRocket"); //right Rocket
		// AutonomousSequences.autoInitFWD("RightPlatformToBackRightRocket"); //right Rocket
		if(!runAuto)
		{
			drivetrainNotifier.startPeriodic(.02);
		}
		else
		{
			pathNotifier.startPeriodic(.02);
			autoNotifier.startPeriodic(.01);
		}
		
		armFollowerNotifier.startPeriodic(.01);
		subsystemsEncodersNotifier.startPeriodic(.05);

		AirCompressor.run();
	}
  
	@Override
	public void autonomousPeriodic() 
	{

		if(mainController.buttonB)
		{
			Drivetrain.stop();
			runAuto = false;
			disabledInit();
			teleopInit();
		}

		if(!runAuto)
			teleopPeriodic();
		else
		{
			SeriesStateMachine.run();
			Arm.run();
			Elevator.run();
			updateJoysticks();
		}	 
	}

	@Override
	public void teleopInit()
	{
		disableAuto();
		Arm.initSensors();
		Elevator.initSensors();
		BallIntake.init();
		MiniShoppingCart.init();
		
		drivetrainNotifier.startPeriodic(.02);
		armFollowerNotifier.startPeriodic(.01);

		subsystemsEncodersNotifier.startPeriodic(.02);
		AirCompressor.run();
	}

	//Teleop Code
	@Override
	public void teleopPeriodic()
	{
		HatchGrabber.run(coController);
		SeriesStateMachine.setControllers(mainController, coController);
		// Arm.updateEncoder();
		// Elevator.updateEncoder();
		SeriesStateMachine.run();
		Arm.run();
		Elevator.run(); 
		BallShooter.runBlink();

		//Drivetrain uses the notifier

		if(SeriesStateMachine.runClimberManually)
		{
			MiniShoppingCart.run(mainController);
		}

		
		
	}

	private static void disableAuto()
	{
		Drivetrain.init();
		Drivetrain.setToBrake();
		autoNotifier.stop();
		pathNotifier.stop();
	}

	@Override
	public void disabledInit()
	{
		drivetrainNotifier.stop();
		Drivetrain.stop();
		pathNotifier.stop();
		subsystemsEncodersNotifier.stop();
		// armFollowerNotifier.stop();
		// stateMachineRunnerNotifier.stop();
	}

	@Override
	public void disabledPeriodic()
	{

	}

	@Override
	public void testInit()
	{
		// Elevator.init();
		// drivetrainNotifier.startPeriodic(.02);
		MiniShoppingCart.init();
		// Arm.init();
		// armFollowerNotifier.startPeriodic(.01);
		// Arm.initSensors();
		// drivetrainNotifier.startPeriodic(.02);
	}
	@Override
	public void testPeriodic()
	{
		MiniShoppingCart.run(mainController);
		mainController.update();

		// BallShooter.stopMotor();
		// HatchGrabber.stopMotor();
		// AirCompressor.run();
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
		if(mainController.rightBumper && mainController.rightJoyStickX < .1)
			vision((Elevator.encoderValue > 27000), VisionMode.kClosest);
		else 
		{
			AutonomousSequences.limelightClimber.limelight.set(VisionMode.kBlack);
			AutonomousSequences.limelightFourBar.limelight.set(VisionMode.kBlack);
			if(Elevator.encoderValue > 27000)
				Drivetrain.customArcadeDrive(mainController.rightJoyStickX * .65, mainController.leftJoyStickY * .6);
			else
				Drivetrain.customArcadeDrive(mainController.rightJoyStickX, mainController.leftJoyStickY);
		}
	}

	

	private void vision(boolean scaleJoy, VisionMode mode)
	{
		double joyY = mainController.leftJoyStickY;
		double joyX = mainController.rightJoyStickX;
		
		VisionController mVision = getLimelight();

		VisionController mVisionDriver = mVision;
		if(mVision.equals(AutonomousSequences.limelightClimber))
			mVisionDriver = AutonomousSequences.limelightFourBar;
		else
			mVisionDriver = AutonomousSequences.limelightClimber;

		if(scaleJoy)
		{
			joyY *= .6;
			joyX *= .65;
		}

		mVision.set(mode);
		if(mode != VisionMode.kBlack)
			mVisionDriver.set(VisionMode.kDriver);

		if(mode == VisionMode.kDriver)
		{
			Drivetrain.customArcadeDrive(joyX, joyY);
		}
		else
		{
			mVision.center();
			Drivetrain.setPercentOutput(mVision.leftSpeed + joyY, mVision.rightSpeed + joyY);
		}
	}

	private VisionController getLimelight()
	{
		if(Arm.aimedState != null)
		{
			//Arm is flipped bwds
			if(Arm.aimedState.encoderVal < Constants.armSRXVerticalStowed)
			{
				//If cargo then its actually forwards
				if(!cargoDetection || SeriesStateMachine.aimedRobotState.equals(ScoringPosition.CARGOLOADINGSTATIONFWD))
					return AutonomousSequences.limelightClimber;
				else
					return AutonomousSequences.limelightFourBar;
			}
			//Arm is forwards
			else
			{
				// if cargo its actually backwards
				if(!cargoDetection || SeriesStateMachine.aimedRobotState.equals(ScoringPosition.CARGOLOADINGSTATIONBWD))
					return AutonomousSequences.limelightFourBar;
				else
					return AutonomousSequences.limelightClimber;
			}
		}
		return AutonomousSequences.limelightClimber;
	}
}