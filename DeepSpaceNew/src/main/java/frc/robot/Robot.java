package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.team3647autonomous.DriveSignal;
import frc.team3647autonomous.MotionProfileDirection;
import frc.team3647autonomous.Odometry;
import frc.team3647autonomous.RamseteFollower;
import frc.team3647autonomous.TrajectoryUtil;
import frc.team3647inputs.*;
import frc.team3647subsystems.*;
import frc.team3647subsystems.Arm.ArmPosition;
import frc.team3647subsystems.Elevator.ElevatorLevel;
import frc.team3647utility.Units;
import jaci.pathfinder.Trajectory;


public class Robot extends TimedRobot 
{
  // private static final String kDefaultAuto = "Default";
  // private static final String kCustomAuto = "My Auto";
  // private String m_autoSelected;
  // private final SendableChooser<String> m_chooser = new SendableChooser<>();

  Joysticks mainController;
  public static Joysticks coController;
  public static Gyro gyro;

  
  public static double[] PIDFleft = new double[4];
  public static double[] PIDFright = new double[4];
  public static double[] PIDF = new double[4];
  public static int mVel = 0;
  public static int mAccel = 0;


  @Override
  public void robotInit() 
  {
    // m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    // m_chooser.addOption("My Auto", kCustomAuto);
    // SmartDashboard.putData("Auto choices", m_chooser);

    mainController = new Joysticks(0);
    coController = new Joysticks(1);
    gyro = new Gyro();
    // TestFunctions.shuffleboardInit();
    // Drivetrain.drivetrainInitialization();

    Drivetrain.initializeSmartDashboardVelAccel();
  }


  @Override
  public void robotPeriodic() 
  {
    gyro.updateGyro();
    updateJoysticks();
  }


  DriveSignal driveSignal;
  Trajectory trajectory;
  RamseteFollower ramseteFollower;
  Trajectory.Segment current;

  @Override
  public void autonomousInit() 
  {
    gyro.resetAngle();
    Drivetrain.drivetrainInitialization();
    Drivetrain.resetEncoders();
    driveSignal = new DriveSignal();
    trajectory = TrajectoryUtil.getTrajectoryFromName("RightPlatformToRightCargoShip");
    ramseteFollower = new RamseteFollower(trajectory, MotionProfileDirection.FORWARD);
    Odometry.getInstance().setInitialOdometry(trajectory);
    Odometry.getInstance().odometryInit();
    ranBackwardsOnce = false;
  }

  double left;
  double right;
  boolean ranBackwardsOnce = false;

  @Override
  public void autonomousPeriodic() 
  {
    driveSignal = ramseteFollower.getNextDriveSignal();
    current = ramseteFollower.currentSegment();

    right = Units.metersToEncoderTicks(driveSignal.getRight() / 10);
    left = Units.metersToEncoderTicks(driveSignal.getLeft() / 10);

    // if(ramseteFollower.isFinished() && !ranBackwardsOnce)
    // {
    //   ramseteFollower = new RamseteFollower(trajectory, MotionProfileDirection.BACKWARD);
    //   Odometry.getInstance().setInitialOdometry(TrajectoryUtil.reversePath(trajectory));
    //   ranBackwardsOnce = true;
    // }

    // ramseteFollower.printOdometry();
    // ramseteFollower.printDeltaDist();
    // ramseteFollower.printCurrentEncoders();

    // System.out.println("Gyro Yaw: " + gyro.getYaw());
    // if (left > right)
    // {
    //   System.out.println("Left Move More");
    // }
    // else if (left < right)
    // {
    //   System.out.println("Right Move More");
    // }
    // else if(left == 0 && right == 0)
    // {
    //   System.out.println("Stopped");
    // }
    // else if (left == right)
    // {
    //   System.out.println("Straight Move");
    // }
    // else
    // {
    //   System.out.println("Something's broken!");
    // }
    Drivetrain.setAutoVelocity(left, right);
    System.out.println("Left Vel: " + (driveSignal.getLeft()) + "\nRight Vel: " + (driveSignal.getRight()));
    System.out.println("Left ticks Vel: " + (left) + "\nRight Vel: " + (right));
  }


  @Override
  public void teleopInit() 
  {
    BallIntake.ballIntakeinitialization();
    Arm.armInitialization();
    Elevator.elevatorInitialization();
    SeriesStateMachine.seriesStateMachineInit();
    // ShoppingCart.shoppingCartInit();
    Drivetrain.drivetrainInitialization();
  }

  @Override
  public void teleopPeriodic() 
  {
    if(mainController.leftBumper || Elevator.elevatorEncoderValue > 27000)
    {
      Drivetrain.customArcadeDrive(mainController.rightJoyStickX * 0.6, mainController.leftJoyStickY * .6, gyro);
    }
    else
    {
      Drivetrain.customArcadeDrive(mainController.rightJoyStickX * 0.65, mainController.leftJoyStickY, gyro);
    }
    
    Arm.runArm();
    Elevator.runElevator();
    HatchGrabber.runHatchGrabber(coController.rightBumper);
    ShoppingCart.runShoppingCart();
    SeriesStateMachine.runSeriesStateMachine(coController, mainController);
    Elevator.printBannerSensor();
    Elevator.printElevatorEncoders();
    System.out.println("ELEVATOR AIMED STATE: " + Elevator.aimedState);
  }

  @Override
  public void disabledInit() 
  {
    TestFunctions.vController.disabledMode();
    TestFunctions.vController2.disabledMode();
    // 
    // Drivetrain.setToCoast();
    // Arm.aimedState = null;
    // Elevator.aimedState = null;
    // SeriesStateMachine.aimedRobotState = null;
  }

  @Override
  public void disabledPeriodic() 
  {
    Arm.armNEO.setIdleMode(IdleMode.kCoast);
  }


  // private Timer secTimer;
  
  @Override
  public void testInit() 
  {
    // Elevator.aimedState = ElevatorLevel.MINROTATE;
    // Drivetrain.drivetrainInitialization();
    // Drivetrain.initializeVelAccel();
    

    // secTimer = new Timer();
    // secTimer.reset();
    // secTimer.start();
    // TestFunctions.shuffleboard();
    //BallIntake.ballIntakeinitialization();
	// BallShooter.ballShooterinitialization();
	// HatchIntake.hatchIntakeInitialization();
  // BallShooter.ballShooterinitialization();
  // Arm.aimedState = ArmPosition.REVLIMITSWITCH;
    Elevator.elevatorInitialization();
  // Elevator.elevatorMaster.enableCurrentLimit(true);
  // Elevator.elevatorMaster.configContinuousCurrentLimit(50);
  // Arm.armInitialization()s; 
  }
  @Override
public void testPeriodic() 
  {

    // Elevator.printBannerSensor();
    // System.out.println(sensor.get());
    //  Elevator.setOpenLoop(mainController.leftJoyStickY);

    // System.out.println("Controller power: " + mainController.leftJoyStickY);
    // System.out.println("Elevator power: " + Elevator.elevatorMaster.getMotorOutputPercent());
    // System.out.println("Elevator voltage: " + Elevator.elevatorMaster.getMotorOutputVoltage());
    // System.out.println("Elevator currnet: " + Elevator.elevatorMaster.getOutputCurrent());
    // Elevator.elevatorMaster.enableCurrentLimit(true);
    // Drivetrain.customArcadeDrive(0, mainController.rightJoyStickY, gyro);
    // Arm.armSRX.set(ControlMode.PercentOutput, -mainController.rightTrigger);
    // Arm.setPosition(10000l);
    // Arm.printArmEncoders();
    // Drivetrain.customArcadeDrive(mainController.rightJoyStickX, mainController.leftJoyStickY, gyro);
    // if(secTimer.get() < 1)
    //   Drivetrain.velAccel();
    // else
      // Drivetrain.customArcadeDrive(mainController.rightJoyStickX, mainController.leftJoyStickY, gyro);
      // if(mainController.leftBumper)
      //   BallIntake.extendIntake();
      // else
      //   BallIntake.retractIntake();
    // System.out.println("Gyro Yaw: " + gyro.getYaw());
    //Vision Code
    // if(mainController.rightBumper)
    // {
    //   TestFunctions.vController.centeringMode();
    //   TestFunctions.vController.center(1, 0.035, 0.15, 0.1);
    //   Drivetrain.setPercentOutput(TestFunctions.vController.leftSpeed + mainController.leftJoyStickY, TestFunctions.vController.rightSpeed + mainController.leftJoyStickY);
    // }
    // else
    // {
    //   TestFunctions.vController.driverMode();
    //   Drivetrain.customArcadeDrive(mainController.rightJoyStickX,mainController.leftJoyStickY, gyro);
    //   TestFunctions.vController.limelight.setToDriver();
    // }
    
    // System.out.println("RIGHT: " + TestFunctions.vController.rightSpeed + " LEFT: " + TestFunctions.vController.leftSpeed);
    // System.out.println(TestFunctions.limelight.getX());
    // HatchIntake.setManualControllerValues(mainController);
    // HatchIntake.runHatchIntakeWrist();
    // HatchIntake.runHatchIntakeClamp(mainController.leftBumper);
    // HatchIntake.printPosition();
    // HatchIntake.printLimitSwitch();
    // BallShooter.printBeamBreak();
    // Elevator.printBannerSensor();
    Elevator.printElevatorEncoders();
    Elevator.setElevatorEncoder();
    // Elevator.runElevator();
    // Arm.runArm();
    // Drivetrain.customArcadeDrive(mainController.rightJoyStickX * 0.7, mainController.leftJoyStickY, gyro);
    // Elevator.runElevator();

    // Elevator.printElevatorEncoders();
    // System.out.println("dPad value: " + coController.dPadValue);
    // HatchGrabber.runHatchGrabber(coController.leftBumper);
    // Arm.printArmLimitSwitches();
    // AirCompressor.runCompressor();
  }

  public void updateJoysticks() 
  {
    mainController.setMainContollerValues();
    coController.setMainContollerValues();
  }
}