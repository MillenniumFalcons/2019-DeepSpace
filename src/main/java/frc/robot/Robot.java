package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.team3647autonomous.DriveSignal;
import frc.team3647autonomous.MotionProfileDirection;
import frc.team3647autonomous.RamseteFollower;
import frc.team3647autonomous.TrajectoryUtil;
import frc.team3647inputs.*;
import frc.team3647subsystems.*;
import jaci.pathfinder.Trajectory;

public class Robot extends TimedRobot {

    Joysticks mainController;
    Joysticks coController;
    public static Gyro gyro;
    // public static Arm arm = new Arm();
    
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

    public static double kP, kI, kD, kF;
    
    @Override
    public void robotInit() 
    {
        // mainController = new Joysticks(0);
        // coController = new Joysticks(1);
        // gyro = new Gyro();
        // TestFunctions.shuffleboard();
        // Drivetrain.drivetrainInitialization();
    }

    @Override
    public void robotPeriodic() 
    {
        // gyro.updateGyro();
        // updateJoysticks();
    }

    @Override
    public void autonomousInit() 
    {
        Drivetrain.selectPIDF(Constants.velocitySlotIdx, Constants.rightVelocityPIDF, Constants.leftVelocityPIDF);
        driveSignal = new DriveSignal();
        trajectory = TrajectoryUtil.getTrajectoryFromName("StraightTenFeet");
        // System.out.println(trajectory.segments[1].x);
        ramseteFollower = new RamseteFollower(trajectory, MotionProfileDirection.FORWARD);
    }
    
    @Override
    public void autonomousPeriodic() 
    {
        driveSignal = ramseteFollower.getNextDriveSignal();
        current = ramseteFollower.currentSegment();

        System.out.println("Velocity: " + ramseteFollower.getVelocity().getLinear() + " X: " + current.x + " Y: " + current.y);
        Drivetrain.setVelocity(driveSignal.getLeft(), driveSignal.getRight());
        // Drivetrain.setVelocity(ramseteFollower.getVelocity().getLinear(), ramseteFollower.getVelocity().getLinear());
    }

    @Override
    public void teleopInit() 
    {

    }
    
    @Override
    public void teleopPeriodic() 
    {
        // HatchGrabber.runHatchGrabber(coController.rightBumper);
    }

    @Override
    public void testInit() 
    {
        // Arm.armInitialization();
        // Elevator.elevatorInitialization();
        // SeriesStateMachine.seriesStateMachineInitialization();
    }

    @Override
    public void testPeriodic()
    {
        // HatchGrabber.runHatchGrabber(coController.rightBumper);
        // HatchIntake.runHatchIntakeWrist(coController);
        // Arm.runArm();
        // Elevator.runElevator();
        // SeriesStateMachine.runSeriesStateMachine(coController);
        // Elevator.printElevatorEncoders();
        // Arm.printArmEncoders();
        System.out.println("LIMF: " + Canifier.cargoBeamBreak());

    }
    @Override
    public void disabledInit() 
    {
        Arm.armSRX.setNeutralMode(NeutralMode.Coast);
    }

    public void updateJoysticks()
    {
        // mainController.setMainContollerValues();
        // coController.setMainContollerValues();
    }
}
