package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3647autonomous.AutonomousSequences;
import frc.team3647autonomous.Odometry;
// import frc.team3647autonomous.Odometry;
import frc.team3647inputs.*;
import frc.team3647subsystems.*;
import frc.team3647subsystems.VisionController.VisionMode;

public class Robot extends TimedRobot 
{
	
	public static PowerDistributionPanel pDistributionPanel;
	public static Joysticks mainController; 
	public static Joysticks coController;
	public static Gyro gyro;
  
	public static Notifier drivetrainNotifier, armFollowerNotifier, autoNotifier, pathNotifier;

	public static boolean cargoDetection;

	@Override
	public void robotInit()
	{
		pDistributionPanel = new PowerDistributionPanel();
		
		AutonomousSequences.limelightClimber.limelight.set(VisionMode.kBlack);
		AutonomousSequences.limelightFourBar.limelight.set(VisionMode.kBlack);
		mainController = new Joysticks(0); 
		coController = new Joysticks(1); 
		gyro = new Gyro();
		
		Drivetrain.init();

		Arm.setToBrake();
		
		armFollowerNotifier = new Notifier(() -> {
			Arm.armNEOFollow();
		});

		drivetrainNotifier = new Notifier(() ->{
			updateJoysticks();
			driveVisionTeleop();
		});

		autoNotifier = new Notifier(() ->{
			Drivetrain.updateEncoders();
			Odometry.getInstance().runOdometry();
		});
		
		pathNotifier = new Notifier(() ->{
			AutonomousSequences.rocketAuto();
			// AutonomousSequences.cargoShipAuto();
		});
		
	}

	@Override
	public void robotPeriodic()	
	{
		gyro.update();
		// updateJoysticks(); Done in drivetrain notifier!
		SmartDashboard.putNumber("Match Timer", DriverStation.getInstance().getMatchTime());
		cargoDetection = BallShooter.cargoDetection();
	}
	
  
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
		ShoppingCart.init();
		Drivetrain.setToBrake();

		// drivetrainNotifier.startPeriodic(.02);
		AutonomousSequences.autoStep = 0;
		armFollowerNotifier.startPeriodic(.01);
		autoNotifier.startPeriodic(.01);
		// AutonomousSequences.autoInitFWD("LeftPlatformToFarSideRocket");
		AutonomousSequences.autoInitFWD("LeftPlatformToLeftRocket");
		// AutonomousSequences.autoInitFWD("LeftPlatformToLeftCargoShipBay1");
		pathNotifier.startPeriodic(.02);
	}
  
	@Override
	public void autonomousPeriodic() 
	{
		// AutonomousSequences.runPath();
		// teleopPeriodic();
		Arm.updateEncoder();
		Elevator.updateEncoder();
		ShoppingCart.updateEncoder();		
		SeriesStateMachine.run();
		Arm.run();
		Elevator.run(); 
	}

	@Override
	public void teleopInit()
	{
		Drivetrain.init();
		Arm.initSensors();
		Elevator.initSensors();

		AirCompressor.run();
		BallIntake.init();

		Drivetrain.setToBrake();

		drivetrainNotifier.startPeriodic(.02);
		armFollowerNotifier.startPeriodic(.01);
		ShoppingCart.init();
	}

	//Teleop Code
	@Override
	public void teleopPeriodic()
	{
		HatchGrabber.run(coController);
		SeriesStateMachine.setControllers(mainController, coController);
		Arm.updateEncoder();
		Elevator.updateEncoder();
		ShoppingCart.updateEncoder();		
		SeriesStateMachine.run();
		Arm.run();
		Elevator.run(); 
		BallShooter.runBlink();
		//Drivetrain uses the notifier

		// ShoppingCart.updateEncoder();
		// if(SeriesStateMachine.elevatorManual)
		// {
		// 	if(mainController.leftJoyStickY > .15)
		// 	{
		// 		ShoppingCart.runSPX(1);
		// 	}
		// 	else if(mainController.leftJoyStickY < -.15)
		// 	{
		// 		ShoppingCart.runSPX(-1);
		// 	}
		// 	else
		// 		ShoppingCart.runSPX(0);
		// }
		
	}

	@Override
	public void disabledInit()
	{
		drivetrainNotifier.stop();
		Drivetrain.stop();
		pathNotifier.stop();
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
		// ShoppingCart.init();
	}
	@Override
	public void testPeriodic()
	{
		// Elevator.updateEncoder();
		Elevator.updateBannerSensor();
		Elevator.printBannerSensor();
		BallShooter.printBeamBreak();
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
			vision((Elevator.encoderValue > 27000), VisionMode.kDriver);
		else if(mainController.rightBumper)
			vision((Elevator.encoderValue > 27000), VisionMode.kClosest);
		else 
		{
			AutonomousSequences.limelightClimber.limelight.set(VisionMode.kBlack);
			AutonomousSequences.limelightFourBar.limelight.set(VisionMode.kBlack);
			if(Elevator.encoderValue > 27000)
				Drivetrain.customArcadeDrive(mainController.rightJoyStickX * .65, mainController.leftJoyStickY * .6);
			else
				Drivetrain.customArcadeDrive(mainController.rightJoyStickX * .7, mainController.leftJoyStickY);
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
				if(!cargoDetection)
					return AutonomousSequences.limelightClimber;
				else
					return AutonomousSequences.limelightFourBar;
			}
			//Arm is forwards
			else
			{
				// if cargo its actually backwards
				if(!cargoDetection)
					return AutonomousSequences.limelightFourBar;
				else
					return AutonomousSequences.limelightClimber;
			}
		}
		return AutonomousSequences.limelightClimber;
	}
}