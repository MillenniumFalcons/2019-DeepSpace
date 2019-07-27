 package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot; 
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3647autonomous.AutonomousSequences;
import frc.team3647autonomous.Odometry;
// import frc.team3647autonomous.Odometry;
import frc.team3647inputs.*;
import frc.team3647subsystems.*;
import frc.team3647subsystems.Arm.ArmPosition;
import frc.team3647subsystems.Elevator.ElevatorLevel;
import frc.team3647subsystems.SeriesStateMachine.ScoringPosition;
import frc.team3647subsystems.VisionController.VisionMode;
import frc.team3647utility.AutoChooser;
import frc.team3647utility.Path;

public class Robot extends TimedRobot 
{
	
	public static PowerDistributionPanel pDistributionPanel;
	public static DriverStation mDriverStation;
	public static Joysticks mainController; 
	public static Joysticks coController;
	public static Gyro gyro;


	//run will control if robot initializes to prevent untrained drivers to use robot when battery is low
	public static boolean run = true;
	public static int brownOutCounter = 0;
  
	public static Notifier drivetrainNotifier, armFollowerNotifier, autoNotifier, pathNotifier, subsystemsEncodersNotifier;

	public static boolean runAuto = false;

	public static boolean cargoDetection;

	private AutoChooser mAutoChooser;

	private Path mPath;

	public enum LastMethod{
		kAuto,
		kTeleop,
		kStarted,
		kTesting;
	}

	public static LastMethod lastMethod = LastMethod.kStarted;

	@Override
	public void robotInit()
	{
		mDriverStation = DriverStation.getInstance();
		lastMethod = LastMethod.kStarted;
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
			gyro.update();
			Odometry.getInstance().runOdometry();
		});
		

		mAutoChooser = new AutoChooser();	
		mAutoChooser.update();
		mPath = new Path(mAutoChooser.getSide(), mAutoChooser.getStruct(), mAutoChooser.getMode());

		AutonomousSequences.limelightClimber.limelight.setUSBStream();
		AutonomousSequences.limelightFourBar.limelight.setUSBStream();

		runAuto = false;
	}

	@Override
	public void robotPeriodic()	
	{
		// AutonomousSequences.limelightClimber.set(VisionMode.kClosestLvl2);
		// System.out.println(AutonomousSequences.limelightClimber.limelight.getArea());
		
		if(isAutonomous())
			gyro.update();
		if(isEnabled())
		{
			coController.update();
		}
		else if(runAuto){
			mAutoChooser.update();
			mPath.update(mAutoChooser.getSide(), mAutoChooser.getStruct(), mAutoChooser.getMode());
			System.out.println("Path running first " + mPath.getIntialPath());
		}

		if(!isEnabled())
		{
			if(RobotController.getBatteryVoltage() <= 12)
			{
				run = false;
			}
		}


		if(RobotController.isBrownedOut())
		{
			brownOutCounter++;
		}

		cargoDetection = BallShooter.cargoDetection();
	}
	
  
	@Override
	public void autonomousInit() 
	{
		try{gyro.resetAngle();}
		catch(NullPointerException e){ gyro = new Gyro(); }

		AutonomousSequences.autoStep = 0;
		
		Drivetrain.init();
		Drivetrain.setToBrake();
		Drivetrain.resetEncoders();

		Arm.init();
		Elevator.init();
		SeriesStateMachine.init();
		BallIntake.init();
		MiniShoppingCart.init();
		
		if(run)
		{
			if(!runAuto)
			{
				drivetrainNotifier.startPeriodic(.02);
			}
			else
			{
				pathNotifier = new Notifier(() ->{
					mPath.run();
				});
				AutonomousSequences.autoInitFWD(mPath.getIntialPath()); //off lvl 2

				pathNotifier.startPeriodic(.02);
				autoNotifier.startPeriodic(.01);
			}			

			armFollowerNotifier.startPeriodic(.01);
			subsystemsEncodersNotifier.startPeriodic(.02);

			AirCompressor.run();
		}
		lastMethod = LastMethod.kAuto;
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
		lastMethod = LastMethod.kAuto;
		System.out.println(Arm.armNEO.getOutputCurrent());
	}

	@Override
	public void teleopInit()
	{
		// disableAuto();
		Arm.initSensors();
		Elevator.initSensors();
		BallIntake.init();
		MiniShoppingCart.init();
		
		if(run)
		{
			drivetrainNotifier.startPeriodic(.02);
			armFollowerNotifier.startPeriodic(.01);

			subsystemsEncodersNotifier.startPeriodic(.02);
			AirCompressor.run();
		}
		lastMethod = LastMethod.kTeleop;
	}

	//Teleop Code
	@Override
	public void teleopPeriodic()
	{
		if(run)
		{
			HatchGrabber.run(coController);
			SeriesStateMachine.setControllers(mainController, coController, isOperatorControl());
			SeriesStateMachine.run();
			Arm.run();
			Elevator.run(); 
			BallShooter.runBlink();
			MiniShoppingCart.run(mainController);
		}
		else
		{
			HatchGrabber.stopMotor();
			drivetrainNotifier.stop();
			Drivetrain.stop();
			armFollowerNotifier.stop();
			AirCompressor.stop();
		}
		lastMethod = LastMethod.kTeleop;
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
		if(pathNotifier != null)
			pathNotifier.stop();
		if(autoNotifier != null)
			autoNotifier.stop();

		if(!lastMethod.equals(LastMethod.kAuto))
		{
			if(drivetrainNotifier != null)
				drivetrainNotifier.stop();

			Drivetrain.stop();

			if(subsystemsEncodersNotifier != null)
				subsystemsEncodersNotifier.stop();
			
			if(armFollowerNotifier != null)
				armFollowerNotifier.stop();

		}
		// stateMachineRunnerNotifier.stop();
		AutonomousSequences.limelightClimber.limelight.setUSBStream();
		AutonomousSequences.limelightClimber.limelight.setUSBStream();

		AutonomousSequences.limelightFourBar.limelight.set(VisionMode.kBlack);
		AutonomousSequences.limelightFourBar.limelight.set(VisionMode.kBlack);
	}

	@Override
	public void disabledPeriodic()
	{
		AutonomousSequences.limelightClimber.limelight.setUSBStream();
		AutonomousSequences.limelightFourBar.limelight.setUSBStream();
	}

	@Override
	public void testInit()
	{
		// Drivetrain.init();
		// SeriesStateMachine.aimedRobotState = ScoringPosition.HATCHL2FORWARDS;
	}
	@Override
	public void testPeriodic()
	{
		// mainController.update();
		// driveVisionTeleop();

		
		mainController.update();

		lastMethod = LastMethod.kTesting;
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
		if(mainController.rightBumper && Math.abs(mainController.rightJoyStickX) < .1)
			vision(SeriesStateMachine.aimedRobotState, Elevator.encoderValue  > 27000);
		else 
		{
			AutonomousSequences.limelightClimber.limelight.set(VisionMode.kBlack);
			AutonomousSequences.limelightFourBar.limelight.set(VisionMode.kBlack);
			Drivetrain.customArcadeDrive(mainController.rightJoyStickX, mainController.leftJoyStickY * .6, mainController.leftJoyStickY < .15, (Elevator.encoderValue > 27000));
		}
	}

	

	private void vision(SeriesStateMachine.ScoringPosition aimedRobotState, boolean scaleInputs)
	{
		double joyY = mainController.leftJoyStickY;
		double joyX = mainController.rightJoyStickX;
		VisionMode mode = VisionMode.kClosestLvl1;
		VisionController mVision = AutonomousSequences.limelightClimber;

		if(aimedRobotState != null)
		{
			mVision = getLimelight(aimedRobotState.armPos);
			
			switch(aimedRobotState.eLevel)
			{
				case CARGO1: //fall through
				case BOTTOM:
					mode = VisionMode.kClosestLvl1;
					break;
				case CARGOL2: //fall through
				case HATCHL2:
					mode = VisionMode.kClosestLvl2;
					break;
				case CARGOL3: //fall through
				case HATCHL3:
					mode = VisionMode.kClosestLvl3;
					break;
			}
		}

		
		boolean scaledJoyVals = false;
		double speedReducer = mVision.speedReducer;

		mVision.set(mode);
		// if(mode != VisionMode.kBlack)
		// 	mVisionDriver.set(VisionMode.kDriver);



		if(mode == VisionMode.kDriver)
		{
			Drivetrain.customArcadeDrive(joyX, joyY, joyY < .15, scaleInputs);
		}
		else
		{
			mVision.center();

			speedReducer = mVision.area != 0 ? Constants.limelightMaxArea / mVision.area : 1;
			if(speedReducer > 1)
			{
				speedReducer = 1;
			}

			//make sure robot drives forward only when joyY is positive, and also when speed reducer is greater than joyY.
			joyY *= speedReducer;

			System.out.println("JoyY: " + joyY + "\nspeedReducer: " + speedReducer + "\nrightSpeed: " + mVision.rightSpeed + "\nleftSpeed: " + mVision.leftSpeed);
			Drivetrain.setPercentOutput(mVision.leftSpeed + joyY, mVision.rightSpeed + joyY, scaleInputs);
		}
	}

	private VisionController getLimelight(Arm.ArmPosition armAimedState) // ( ) ball shooter, >< hatch intake, ---- arm, encoderVal is where the ball intake is.
	{
		if(armAimedState != null)
		{
			if(armAimedState.encoderVal < Constants.armSRXVerticalStowed) //Arm is, going to be, forwards )---->
			{
				// if there is no cargo and the aimed state isn't cargo intake, use the front limelight
				if(!cargoDetection && !SeriesStateMachine.aimedRobotState.equals(ScoringPosition.CARGOLOADINGSTATIONBWD))
				{
					return AutonomousSequences.limelightClimber;
				}
				//If there is cargo, or cargo loading station backwards is the aimed state, use the rear limelight
				return AutonomousSequences.limelightFourBar;
			}
			
			else //Arm is backwards <----(
			{
				// If there is no cargo and the state isn't cargo forwards, use rear limelight
				if(!cargoDetection && !SeriesStateMachine.aimedRobotState.equals(ScoringPosition.CARGOLOADINGSTATIONFWD))
				{
					return AutonomousSequences.limelightFourBar;
				}
				//If there is cargo, or state is cargo loading station front, use front limelight
				return AutonomousSequences.limelightClimber;
			}
		}

		// if arm aimed state is null, return front limelight
		return AutonomousSequences.limelightClimber;
	}
}