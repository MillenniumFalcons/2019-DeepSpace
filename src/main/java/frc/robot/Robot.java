package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

import frc.team3647inputs.*;
import frc.team3647subsystems.*;

public class Robot extends TimedRobot 
{
   
    Joysticks mainController;
    Joysticks coController;
    public static Gyro gyro;
    public static Arm arm = new Arm();

	public static double kP = 0;
	public static double kI = 0;
	public static double kD = 0;
	public static double kF = .35;
	public static int mVel = 0;
	public static int mAccel = 0;
    
 
    @Override
    public void robotInit() 
    {
        mainController = new Joysticks(0);
        coController = new Joysticks(1);
        //gyro = new Gyro();
        TestFunctions.shuffleboard();
    }

    @Override
    public void robotPeriodic() 
    {
        //gyro.updateGyro();
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
       // Arm.armInitialization();
       Elevator.elevatorInitialization();

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
        TestFunctions.updatePIDFMM();
        arm.configurePIDFMM(kP, kI, kD, kF, mVel, mAccel);
        // System.out.println("Elevator sensor " + Elevator.getLimitSwitch());
        // System.out.println("Elevator sensor " + Elevator.elevatorEncoderValue);
        // System.out.println("Elevator aimed state: " + Elevator.aimedState);
        // Elevator.runElevator(mainController);
        arm.runArm(mainController);
        // if(mainController.rightTrigger > .15)
        //     Elevator.stopElevator();
        // Elevator.testElevatorCurrent();

        // System.out.println("dPad Up: " + mainController.dPadUp);
        // System.out.println("dPad down: " + mainController.dPadDown);
        // System.out.println("dPad left: " + mainController.dPadLeft);
        // System.out.println("dPad right: " + mainController.dPadRight);

        // System.out.println("POV: " + mainController.dPadValue);


    }

    public void updateJoysticks()
    {
        mainController.setMainContollerValues();
        coController.setMainContollerValues();
    }
}
