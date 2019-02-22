package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
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
    TestFunctions.shuffleboard();
    Drivetrain.drivetrainInitialization();
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
    // Drivetrain.selectPIDF(Constants.velocitySlotIdx, Constants.rightVelocityPIDF, Constants.leftVelocityPIDF);
    driveSignal = new DriveSignal();
    trajectory = TrajectoryUtil.getTrajectoryFromName("StraightTenFeet");
    ramseteFollower = new RamseteFollower(trajectory, MotionProfileDirection.FORWARD);
  }

  double left;
  double right;

  @Override
  public void autonomousPeriodic() 
  {
    driveSignal = ramseteFollower.getNextDriveSignal();
    current = ramseteFollower.currentSegment();

    left = Units.metersToEncoderTicks(driveSignal.getLeft())/10;
    right = Units.metersToEncoderTicks(driveSignal.getRight())/10;

    System.out.println("{Left Drive Signal: " + left + " Right Drive Signal: " + right + "}");
    // Drivetrain.setAutoVelocity(left, right);
  }


  @Override
  public void teleopInit() 
  {
    BallIntake.ballIntakeinitialization();
    Arm.armInitialization();
    Elevator.elevatorInitialization();
    SeriesStateMachine.seriesStateMachineInitialization();
    HatchIntake.hatchIntakeInitialization();
    Drivetrain.drivetrainInitialization();
  }

  // public void runPCJoy()
  // {
  //   if(mainController.dPadUp)
  //   {
  //     Drivetrain.customArcadeDrive(0, .5, gyro);
  //   }
  //   else if(mainController.dPadDown)
  //   {
  //     Drivetrain.customArcadeDrive(0, -.5, gyro);
  //   }
  //   else if(mainController.dPadLeft)
  //   {
  //     Drivetrain.customArcadeDrive(-.5, 0, gyro);
  //   }
  //   else if(mainController.dPadRight)
  //   {
  //     Drivetrain.customArcadeDrive(.5, 0, gyro);
  //   }
  //   else
  //   {
  //     Drivetrain.stop();
  //   }
  // }


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
    
    HatchGrabber.runHatchGrabber(coController.rightBumper);
    Arm.runArm();
    Elevator.runElevator();
    Elevator.printElevatorEncoders();
    HatchIntake.runHatchIntakeWrist();
    HatchIntake.runHatchIntakeClamp(coController.leftBumper);
    SeriesStateMachine.runSeriesStateMachine(coController, mainController);
  }

  @Override
  public void disabledInit() 
  {
    TestFunctions.vController.limelight.setToVison();
    TestFunctions.vController2.limelight.setToVison();
    TestFunctions.vController.limelight.ledOff();
    TestFunctions.vController2.limelight.ledOff();
    // Arm.armNEO.setIdleMode(IdleMode.kCoast);
    // Drivetrain.setToCoast();
    // Arm.aimedState = null;
    // Elevator.aimedState = null;
    // SeriesStateMachine.aimedRobotState = null;
  }


  @Override
  public void testInit() 
  {
    //Elevator.elevatorInitialization();
    // Elevator.aimedState = ElevatorLevel.MINROTATE;
    // Drivetrain.drivetrainInitialization();
    // TestFunctions.shuffleboard();
    //BallIntake.ballIntakeinitialization();
	// BallShooter.ballShooterinitialization();
	// HatchIntake.hatchIntakeInitialization();
  // BallShooter.ballShooterinitialization();
  // Arm.aimedState = ArmPosition.REVLIMITSWITCH;
  Elevator.elevatorInitialization();    
  }
  @Override
  public void testPeriodic() 
  {
    // if(mainController.rightBumper)
    // {
    //   TestFunctions.vController.limelight.setToVison();
    //   TestFunctions.vController.center(1, 0.035, 0.15, 0.1);
    //   Drivetrain.setPercentOutput(TestFunctions.vController.leftSpeed + mainController.leftJoyStickY, TestFunctions.vController.rightSpeed + mainController.leftJoyStickY);
    // }
    // else
    // {
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
    // Elevator.printElevatorEncoders();
    // Elevator.setElevatorEncoder();
    // Elevator.runElevator();
    // Arm.runArm();
    // Drivetrain.customArcadeDrive(mainController.rightJoyStickX * 0.7, mainController.leftJoyStickY, gyro);
    // Elevator.runElevator();
    Elevator.setOpenLoop(mainController.leftJoyStickY * .5);
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