package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
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

  DriveSignal driveSignal;
  Trajectory trajectory;
  RamseteFollower ramseteFollower;
  Trajectory.Segment current;

  public static double kPleft = 0.1;
  public static double kIleft = 0;
  public static double kDleft = 0;
  public static double kFleft = 0;

  public static double kPright = 0.1;
  public static double kIright = 0;
  public static double kDright = 0;
  public static double kFright = 0;
  public static int mVel = 0;
  public static int mAccel = 0;

  public static double kP = 0, kI = 0, kD = 0, kF = 0;


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


  @Override
  public void autonomousInit() 
  {
    // m_autoSelected = m_chooser.getSelected();
    // // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    // System.out.println("Auto selected: " + m_autoSelected);

    Drivetrain.selectPIDF(Constants.velocitySlotIdx, Constants.rightVelocityPIDF, Constants.leftVelocityPIDF);
    driveSignal = new DriveSignal();
    trajectory = TrajectoryUtil.getTrajectoryFromName("StraightTenFeet");
    // System.out.println(trajectory.segments[1].x);
    ramseteFollower = new RamseteFollower(trajectory, MotionProfileDirection.FORWARD);
  }

  @Override
  public void autonomousPeriodic() 
  {
    // switch (m_autoSelected) 
    // {
    // case kCustomAuto:
    //   // Put custom auto code here
    //   break;
    // case kDefaultAuto:
    // default:
    //   // Put default auto code here
    //   break;
    // }

    driveSignal = ramseteFollower.getNextDriveSignal();
    current = ramseteFollower.currentSegment();

    System.out.println("Velocity: " + ramseteFollower.getVelocity().getLinear() + " X: " + current.x + " Y: " + current.y);
    Drivetrain.setVelocity(driveSignal.getLeft(), driveSignal.getRight());
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


  @Override
  public void teleopPeriodic() 
  {
    Drivetrain.customArcadeDrive(mainController.rightJoyStickX, mainController.leftJoyStickY, gyro);
    HatchGrabber.runHatchGrabber(coController.rightBumper);
    Arm.runArm();
    Elevator.runElevator();
    Elevator.printElevatorEncoders();
    HatchIntake.runHatchIntakeWrist();
    HatchIntake.runHatchIntakeClamp(coController.leftBumper);
    SeriesStateMachine.runSeriesStateMachine(coController);
  }

  @Override
  public void disabledInit() 
  {
    Arm.armNEO.setIdleMode(IdleMode.kCoast);
    Drivetrain.setToCoast();
  }


  @Override
  public void testInit() 
  {
    // Elevator.elevatorInitialization();
    // Elevator.aimedState = ElevatorLevel.MINROTATE;
    // Arm.armInitialization();
    // TestFunctions.shuffleboard();
    //BallIntake.ballIntakeinitialization();
	// BallShooter.ballShooterinitialization();
	// HatchIntake.hatchIntakeInitialization();
  // BallShooter.ballShooterinitialization();
  // Arm.aimedState = ArmPosition.REVLIMITSWITCH;
    
  }
  @Override
  public void testPeriodic() 
  {
	//   HatchIntake.setManualControllerValues(mainController);
	//   HatchIntake.runHatchIntakeWrist();
	//   HatchIntake.runHatchIntakeClamp(mainController.leftBumper);
	//   HatchIntake.printPosition();
	//   HatchIntake.printLimitSwitch();
  // BallShooter.printBeamBreak();
    // Elevator.printBannerSensor();
    // Elevator.printElevatorEncoders();
    // Elevator.setElevatorEncoder();
    // Elevator.runElevator();
    // Arm.runArm();
    // System.out.println("dPad value: " + coController.dPadValue);
    HatchGrabber.runHatchGrabber(coController.leftBumper);
  }

  public void updateJoysticks() 
  {
    mainController.setMainContollerValues();
    coController.setMainContollerValues();
  }
}