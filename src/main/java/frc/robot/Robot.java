package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

import frc.team3647inputs.*;
import frc.team3647subsystems.*;

public class Robot extends TimedRobot 
{
   
    Joysticks mainController;
    Joysticks coController;
    public static Gyro gyro;
    
 
    @Override
    public void robotInit() 
    {
        mainController = new Joysticks(0);
        coController = new Joysticks(1);
        gyro = new Gyro();
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
        HatchIntake.hatchIntakeInitialization();
        BallIntake.ballIntakeinitialization();
        BallShooter.ballShooterinitialization();
       // Arm.armInitialization();
    }

    @Override
    public void testPeriodic()
    {
        // gyro.printAngles();
        Drivetrain.customArcadeDrive(mainController.leftJoyStickY, mainController.rightJoyStickX, gyro);
        // BallIntake.runSmartBallIntake(coController.leftTrigger, coController.leftBumper);
        // HatchIntake.runHatchIntakeWrist(coController);
        // HatchIntake.printPosition();
        // HatchGrabber.runHatchGrabber(coController.rightBumper);
        // Arm.moveManual(coController.rightJoyStickY);
    }

    public void updateJoysticks()
    {
        mainController.setMainContollerValues();
        coController.setMainContollerValues();
    }
}
