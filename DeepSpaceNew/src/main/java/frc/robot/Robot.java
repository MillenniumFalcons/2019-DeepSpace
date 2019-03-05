package frc.robot; 

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Notifier; 
import edu.wpi.first.wpilibj.TimedRobot; 
import edu.wpi.first.wpilibj.Timer; 
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 
import frc.team3647autonomous.AutonomousSequences; 
import frc.team3647autonomous.Odometry; 
import frc.team3647inputs. * ; 
import frc.team3647subsystems. * ; 

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
		matchTimer = new Timer();
		mainController = new Joysticks(0); 
		coController = new Joysticks(1); 
		gyro = new Gyro(); 
		// TestFunctions.shuffleboardInit();
		// Drivetrain.drivetrainInitialization();

		Drivetrain.initializeSmartDashboardVelAccel(); 
		Arm.armNEO.setIdleMode(IdleMode.kBrake); 
		autoTimer = new Timer();
		CameraServer.getInstance().startAutomaticCapture(0);
		CameraServer.getInstance().startAutomaticCapture(1);
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
		// The NEO takes the Motor-output in percent from the SRX and since SRX
		// values are using motion-magic, it "follows" the SRX
		// autoNotifier = new Notifier(() ->
		// {
		// 	Arm.armNEOFollow();
		// 	Odometry.getInstance().runOdometry();
		// });
		// autoNotifier.startPeriodic(0.01);

		// gyro.resetAngle(); 
		// Drivetrain.drivetrainInitialization(); 
		// Drivetrain.resetEncoders(); 
		// AirCompressor.runCompressor(); 
		// AutonomousSequences.autoInit("Level2RightToCargoRight"); 
		// autoTimer.reset(); 
		// autoTimer.start(); 

		matchTimer.reset();
		matchTimer.start();
		// BallIntake.ballIntakeinitialization(); 
		// Arm.armInitialization(); 
		// Elevator.elevatorInitialization(); 
		// SeriesStateMachine.seriesStateMachineInit(); 
		// // ShoppingCart.shoppingCartInit(); 
		// Drivetrain.drivetrainInitialization(); 

		// teleopNotifier = new Notifier(() -> 
    	// {
		// 	Arm.armNEOFollow(); 
		// }); 
		// teleopNotifier.startPeriodic(.01);
	}


	@Override
	public void autonomousPeriodic()
	{
		// teleopPeriodic();
	}

	@Override
	public void teleopInit()
	{
		// BallIntake.ballIntakeinitialization(); 
		// Arm.armInitialization(); 
		// Elevator.elevatorInitialization(); 
		// SeriesStateMachine.seriesStateMachineInit(); 
		// ShoppingCart.shoppingCartInit(); 
		// Drivetrain.drivetrainInitialization(); 

		// teleopNotifier = new Notifier(() -> 
    //     {
		// 	Arm.armNEOFollow(); 
		// }); 
		// teleopNotifier.startPeriodic(.01); 

	}

	//Teleop Code
	@Override
	public void teleopPeriodic()
	{
		teleop();
		Arm.runArm(); 
		Elevator.runElevator(); 
		HatchGrabber.runHatchGrabber(coController.rightBumper); 
		SeriesStateMachine.setControllers(mainController, coController); 
		SeriesStateMachine.runSeriesStateMachine(); 
	}

	@Override
	public void disabledInit()
	{
		
		// AutonomousSequences.limelightTop.rightMost();
		// AutonomousSequences.limelightBottom.rightMost();
		// AutonomousSequences.limelightTop.disabledMode(); 
		// AutonomousSequences.limelightBottom.disabledMode(); 
		// TestFunctions.vController.disabledMode();
		// Arm.disableArm();
		// Drivetrain.setToCoast();
		// Odometry.getInstance().closeOdoThread();
		// Elevator.aimedState = null;
		// SeriesStateMachine.aimedRobotState = null;
		matchTimer.stop();
		matchTimer.reset();
	}

	@Override
	public void disabledPeriodic()
	{
		// AutonomousSequences.limelightTop.rightMost();
		// AutonomousSequences.limelightBottom.rightMost();

		// Arm.armNEO.setIdleMode(IdleMode.kBrake);
		// Drivetrain.setToCoast();
		// AutonomousSequences.limelightTop.disabledMode(); 
		// AutonomousSequences.limelightBottom.disabledMode(); 
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

		// ShoppingCart.shoppingCartInit();
		AutonomousSequences.limelightClimber.leftMost();
		AutonomousSequences.limelightFourBar.leftMost();
	}


	@Override
	public void testPeriodic()
	{
		AutonomousSequences.limelightClimber.leftMost();
		if(mainController.rightBumper)
		{
			switch(visionCase)
			{
			case 0:
				AutonomousSequences.limelightClimber.center(0.075);
				Drivetrain.setPercentOutput(-AutonomousSequences.limelightClimber.rightSpeed + mainController.leftJoyStickY, -AutonomousSequences.limelightClimber.leftSpeed + mainController.leftJoyStickY);
				if(AutonomousSequences.limelightClimber.area > 6)
						visionCase = 1;
					break;
				
			case 1:
				Drivetrain.customArcadeDrive(mainController.rightJoyStickX * .7, mainController.leftJoyStickY, gyro);
						
			}
			
		}
		else if(mainController.leftBumper)
		{
			new VictorSPX(1).set(ControlMode.PercentOutput, mainController.leftJoyStickY);
		}
		else 
		{
			visionCase = 0;
			Drivetrain.customArcadeDrive(mainController.rightJoyStickX * .7, mainController.leftJoyStickY, gyro); 
		}
		// AirCompressor.runCompressor();

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
		// Elevator.setOpenLoop(mainController.rightJoyStickY); 
		// ShoppingCart.setShoppinCartEncoder();
		// ShoppingCart.printPosition();
		// ShoppingCart.shoppingCartMotor.set(ControlMode.PercentOutput, mainController.leftJoyStic kY);
	}

	public void updateJoysticks()
	{
		mainController.setMainContollerValues();
		coController.setMainContollerValues();
	}
	
	public void teleop()
	{
		if (Elevator.elevatorEncoderValue > 27000) 
		{
			Drivetrain.customArcadeDrive(mainController.rightJoyStickX * .65, mainController.leftJoyStickY * .6, gyro);
		} 
		else if (Arm.armEncoderValue > Constants.armSRXVerticalStowed) //Hatch intake above fourbar
		{
			SmartDashboard.putString("ARM ORIENTATION", "HATCH BACKWARDS");
			teleopVisionBackward(AutonomousSequences.limelightFourBar, AutonomousSequences.limelightClimber, .075);

		} 
		else if (Arm.armEncoderValue < Constants.armSRXVerticalStowed)//Hatch intake above climber
		{
			SmartDashboard.putString("ARM ORIENTATION", "HATCH FORWARDS");
			teleopVisionForward(AutonomousSequences.limelightClimber, AutonomousSequences.limelightFourBar, .075);
		} 
		else 
		{
			AutonomousSequences.limelightClimber.driverMode();
			AutonomousSequences.limelightFourBar.driverMode();
			Drivetrain.customArcadeDrive(mainController.rightJoyStickX * .7, mainController.leftJoyStickY, gyro);
		}
	}
	
	
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
	
	public void teleopVisionBackward(VisionController camOne, VisionController camTwo, double threshold) //.075
	{
		double joyY = mainController.leftJoyStickY;
		camTwo.driverFlipped();
		if (mainController.rightBumper) 
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
		else if (mainController.leftBumper) 
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
		else 
		{
			visionCase = 0;
			Drivetrain.customArcadeDrive(mainController.rightJoyStickX * .7, mainController.leftJoyStickY, gyro);
		}
	}
}