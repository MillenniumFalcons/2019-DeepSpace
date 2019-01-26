package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.team3647inputs.*;
import frc.team3647pistons.AirCompressor;
import frc.team3647pistons.IntakeBall;
import frc.team3647pistons.IntakeHatch;
import frc.team3647subsystems.Drivetrain;


public class Robot extends TimedRobot 
{

    Joysticks mainController;
    Joysticks coController;
    Gyro gyro;
    
    @Override
    public void robotInit() 
    {
        mainController = new Joysticks(0);
        coController = new Joysticks(1);
        gyro = new Gyro();
        Drivetrain.drivetrainInitialization();
    }

    @Override
    public void robotPeriodic() 
    {
        
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
        testDrivetrain1();
    }


    //testing ball intake and drivetrain 01/26/2019
    public void testDrivetrain1()
    {
        boolean ball = false;
        Drivetrain.customArcadeDrive(mainController.leftJoyStickX, mainController.leftJoyStickY, gyro);
        if(mainController.buttonX)
        {
            if(ball == false)
            {
                IntakeBall.openIntake();
                ball = true;
            }
            if(mainController.rightBumper)
                IntakeBall.setSpeed(.5);

            else if(mainController.leftBumper)
                IntakeBall.setSpeed(-.5);
                
            else
                IntakeBall.setSpeed(0);
        }
        else
        {
            IntakeBall.closeIntake();
            ball = false;
        }
        if(mainController.buttonA)
        {
            IntakeHatch.openIntake();
        }
        else
        {
            IntakeHatch.closeIntake();
        }
    }
    
    @Override
    public void testPeriodic() 
    {
        AirCompressor.runCompressor();
        // mainController.vibrate(1, .9, 1);
    }

    public void updateControllers()
    {

    }
}
