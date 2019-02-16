package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
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
    // Joysticks coController;
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

    @Override
    public void robotInit() 
    {
        mainController = new Joysticks(0);
<<<<<<< HEAD
        coController = new Joysticks(1);
=======
        // coController = new Joysticks(1);
>>>>>>> c8c0efb7850b56a5edaeefbfd09246a8776fc4a0
        gyro = new Gyro();
        TestFunctions.shuffleboard();
        Drivetrain.drivetrainInitialization();
    }

    @Override
<<<<<<< HEAD
    public void robotPeriodic() 
    {
=======
    public void robotPeriodic() {
>>>>>>> c8c0efb7850b56a5edaeefbfd09246a8776fc4a0
        gyro.updateGyro();
        updateJoysticks();
    }

    @Override
    public void autonomousInit() 
    {
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

    }

    @Override
    public void testInit() 
    {
<<<<<<< HEAD
        //HatchIntake.hatchIntakeInitialization();
        //BallIntake.ballIntakeinitialization();
        //BallShooter.ballShooterinitialization();
        Arm.armInitialization();
        Elevator.elevatorInitialization();
        SeriesStateMachine.seriesStateMachineInitialization();
       //Arm.setToBrake();


=======
        // HatchIntake.hatchIntakeInitialization();
        // BallIntake.ballIntakeinitialization();
        // BallShooter.ballShooterinitialization();
        // Arm.armInitialization();
        // Elevator.elevatorInitialization();
        TestFunctions.shuffleboard();
        Drivetrain.resetEncoders();
>>>>>>> c8c0efb7850b56a5edaeefbfd09246a8776fc4a0
    }

    @Override
    public void testPeriodic()
    {
<<<<<<< HEAD
       // gyro.printAngles();
        //Drivetrain.customArcadeDrive(mainController.leftJoyStickY, mainController.rightJoyStickX, gyro);
        // BallIntake.runSmartBallIntake(coController.leftTrigger, coController.leftBumper);
=======
        // gyro.printAngles();
        Drivetrain.velocityDrive(mainController.rightJoyStickX, mainController.leftJoyStickY, gyro);
        Drivetrain.configurePIDFMM(kPright, kIright, kDright, kFright, kPleft, kIleft, kDleft, kFleft, mVel, mAccel);
        // System.out.println(Drivetrain.leftSRX.getSelectedSensorPosition() - Drivetrain.rightSRX.getSelectedSensorPosition());
        System.out.println(Drivetrain.leftSRX.getSelectedSensorVelocity());
        // BallIntake.runSmartBallIntake(coController.leftTrigger,
        // coController.leftBumper);
>>>>>>> c8c0efb7850b56a5edaeefbfd09246a8776fc4a0
        // HatchIntake.runHatchIntakeWrist(coController);
        // HatchIntake.printPosition();
        // HatchGrabber.runHatchGrabber(coController.rightBumper);
        // Arm.moveManual(coController.rightJoyStickY);
        //TestFunctions.updatePIDFMM();
        //Elevator.configurePIDFMM(kP, kI, kD, kF, mVel, mAccel);
        //Elevator.setManualController(mainController);
        //Arm.setManualController(mainController);
        Elevator.runElevator();
        Arm.runArm();
        
        // System.out.println("ARM CS: " + Arm.currentState + " Arm AS: " + Arm.aimedState);
        // System.out.println("ELEV CS: " + Elevator.currentState + " Elev AS: " + Elevator.aimedState);
        SeriesStateMachine.runSeriesStateMachine(mainController);
        Elevator.printElevatorEncoders();
<<<<<<< HEAD
        Arm.printArmEncoders();
=======
		//System.out.println("Is NEO following: " + Arm.armNEO.isFollower());
        //Arm.printPercentOutput();
        //System.out.println("CCL: " + Arm.armEncoderCCL);

        // if(mainController.rightTrigger > .15)
        // Elevator.stopElevator();
        // Elevator.testElevatorCurrent();

        // System.out.println("dPad Up: " + mainController.dPadUp);
        // System.out.println("dPad down: " + mainController.dPadDown);
        // System.out.println("dPad left: " + mainController.dPadLeft);
        // System.out.println("dPad right: " + mainController.dPadRight);

        // System.out.println("POV: " + mainController.dPadValue);
>>>>>>> c8c0efb7850b56a5edaeefbfd09246a8776fc4a0
    }
    @Override
    public void disabledInit() {
        Arm.armSRX.setNeutralMode(NeutralMode.Coast);
    }

    

    public void updateJoysticks()
    {
        mainController.setMainContollerValues();
        // coController.setMainContollerValues();
    }
}
