package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3647autonomous.AutonomousSequences;
// import frc.team3647autonomous.Odometry;
import frc.team3647inputs.*;
import frc.team3647subsystems.*;
import frc.team3647subsystems.SeriesStateMachine.ScoringPosition;

public class Robot extends TimedRobot 
{
	
	public static PowerDistributionPanel pDistributionPanel;
	public static Joysticks mainController; 
	public static Joysticks coController;
	public static Gyro gyro;
  
	public static Notifier drivetrainNotifier, armFollowerNotifier; //autoNotifier;

	public static boolean cargoDetection;

	@Override
	public void robotInit()
	{
		pDistributionPanel = new PowerDistributionPanel();
		
		AutonomousSequences.limelightClimber.limelight.setToBlack();
		AutonomousSequences.limelightFourBar.limelight.setToBlack();
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

		// autoNotifier = new Notifier(() ->{
		// 	Drivetrain.updateEncoders();
		// 	Odometry.getInstance().runOdometry();
		// });
		
		
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

		Drivetrain.setToBrake();

		drivetrainNotifier.startPeriodic(.02);
		armFollowerNotifier.startPeriodic(.01);
	}
  
	@Override
	public void autonomousPeriodic() 
	{
		// AutonomousSequences.runPath();
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

		Drivetrain.setToBrake();

		drivetrainNotifier.startPeriodic(.02);
		armFollowerNotifier.startPeriodic(.01);
		ShoppingCart.init();
	}

	//Teleop Code
	@Override
	public void teleopPeriodic()
	{
		// HatchGrabber.run(coController);
		SeriesStateMachine.setControllers(mainController, coController);
		Arm.updateEncoder();
		Elevator.updateEncoder();
		ShoppingCart.updateEncoder();		
		SeriesStateMachine.run();
		Arm.run();
		Elevator.run(); 
		BallShooter.runBlink();
		//Drivetrain uses the notifier
		ShoppingCart.updateEncoder();
		if(SeriesStateMachine.elevatorManual && mainController.leftJoyStickY > .15)
		{
			ShoppingCart.runSPX(1);
		}
		ShoppingCart.printPosition();
	}

	@Override
	public void disabledInit()
	{
		drivetrainNotifier.stop();
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
		// drivetrainNotifier.startPeriodic(.02);
		ShoppingCart.init();
	}
	@Override
	public void testPeriodic()
	{
		ShoppingCart.updateEncoder();
		ShoppingCart.printPosition();
		// HatchGrabber.run(coController);
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
			vision((Elevator.encoderValue > 27000), VisionContour.kDriver);
		else if(mainController.rightBumper)
			vision((Elevator.encoderValue > 27000), VisionContour.kClosest);
		else 
		{
			AutonomousSequences.limelightClimber.limelight.setToBlack();
			AutonomousSequences.limelightFourBar.limelight.setToBlack();
			if(Elevator.encoderValue > 27000)
				Drivetrain.customArcadeDrive(mainController.rightJoyStickX * .65, mainController.leftJoyStickY * .6);
			else
				Drivetrain.customArcadeDrive(mainController.rightJoyStickX * .7, mainController.leftJoyStickY);
		}
	}

	// private VisionSide getVisionSide()
	// {
	// 	boolean armAfterVerticalStowed = Arm.encoderValue > Constants.armSRXVerticalStowed;
	// 	if(Arm.aimedState != null)
	// 	{
	// 		if(Arm.aimedState.equals(ArmPosition.FLATBACKWARDS) || 
	// 			armAfterVerticalStowed || 
	// 			Arm.aimedState.equals(ArmPosition.CARGOL3FRONT)
	// 		  )
	// 			return VisionSide.kBackwards;
			
	// 		else if(Arm.aimedState.equals(ArmPosition.FLATFORWARDS) || 
	// 				!armAfterVerticalStowed || 
	// 				Arm.aimedState.equals(ArmPosition.CARGOL3BACK)
	// 	  		   )
	// 			return VisionSide.kForwards;


	// 		return VisionSide.kForwards;
	// 	}
	// 	else
	// 	{
	// 		if(armAfterVerticalStowed)
	// 			return VisionSide.kBackwards;
	// 		return VisionSide.kForwards;
	// 	}	
	// }
	
	// private enum VisionSide
	// {
	// 	kBackwards,
	// 	kForwards,
	// 	kTop,
	// 	kBottom,
	// 	kUnknown,
	// }
	private enum VisionContour
	{
		kRight,
		kLeft,
		kClosest,
		kDriver,
		kBlack
	}

	private void vision(boolean scaleJoy, VisionContour contour)
	{
		double joyY = mainController.leftJoyStickY;
		double joyX = mainController.rightJoyStickX;
		VisionController mVision = getLimelight();

		if(scaleJoy)
		{
			joyY *= .6;
			joyX *= .65;
		}

		switch(contour)
		{
			case kClosest:
				mVision.closestMode();
				break;
			case kDriver:
				mVision.driverMode();
				break;
			case kBlack:
				mVision.disabledMode();
				break;
			case kRight:
				mVision.rightMost();
				break;
			case kLeft:
				mVision.leftMost();
				break;
		}

		if(contour == VisionContour.kDriver)
		{
			Drivetrain.customArcadeDrive(joyX, joyY);
		}
		else
		{
			mVision.center(Constants.limelightThreshold);
			Drivetrain.setPercentOutput(mVision.leftSpeed + joyY, mVision.rightSpeed + joyY);
		}
	}

	private VisionController getLimelight()
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
}