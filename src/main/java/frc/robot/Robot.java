package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.TimedRobot;

import frc.team3647inputs.*;
import frc.team3647subsystems.*;
import frc.team3647subsystems.Arm.ArmPosition;
import frc.team3647subsystems.Elevator.ElevatorLevel;

public class Robot extends TimedRobot 
{
   
    public static Joysticks mainController;
    Joysticks coController;
    public static Gyro gyro;

	public static double kP = 0.5;
	public static double kI = 0;
	public static double kD = 0;
	public static double kF = 0;
	public static int mVel = 300;
	public static int mAccel = 150;
    
 
    @Override
    public void robotInit() 
    {
        mainController = new Joysticks(0);
        coController = new Joysticks(1);
        gyro = new Gyro();
        TestFunctions.shuffleboard();
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

    }
    
    @Override
    public void autonomousPeriodic() 
    {
      
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
        //HatchIntake.hatchIntakeInitialization();
        //BallIntake.ballIntakeinitialization();
        //BallShooter.ballShooterinitialization();
        Arm.armInitialization();
        Elevator.elevatorInitialization();
        SeriesStateMachine.seriesStateMachineInitialization();
       //Arm.setToBrake();


    }

    @Override
    public void testPeriodic()
    {
       // gyro.printAngles();
        //Drivetrain.customArcadeDrive(mainController.leftJoyStickY, mainController.rightJoyStickX, gyro);
        // BallIntake.runSmartBallIntake(coController.leftTrigger, coController.leftBumper);
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
        Arm.printArmEncoders();
    }
    @Override
    public void disabledInit() {
        Arm.armSRX.setNeutralMode(NeutralMode.Coast);
    }

    

    public void updateJoysticks()
    {
        mainController.setMainContollerValues();
        coController.setMainContollerValues();
    }
}
