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
    public static Encoders encoders;
    public static Gyro gyro;
    public static final Drivetrain drivetrain = new Drivetrain();
    
    @Override
    public void robotInit() 
    {
        encoders = new Encoders();
        mainController = new Joysticks(0);
        coController = new Joysticks(1);
        gyro = new Gyro();
        drivetrain.drivetrainInitialization();
    }

    @Override
    public void robotPeriodic() 
    {
        updateControllers();
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
        // System.out.println(mainController.rightJoyStickY);
        // System.out.println(mainController.leftJoyStickX);
        drivetrain.leftSRX.set(.35);
        System.out.println(drivetrain.leftSRX.getMotorOutputPercent());
        // drivetrain.setPercentOutput(.2,0);
    }


    //testing ball intake and drivetrain 01/26/2019
    public void testDrivetrain1()
    {
        drivetrain.customArcadeDrive(mainController.leftJoyStickY, mainController.rightJoyStickX, gyro);
        System.out.println("Bumpers " + mainController.leftBumper + " " + mainController.rightBumper);
        System.out.println("Triggers " + mainController.leftTrigger + " " + mainController.rightTrigger);
        if(mainController.rightTrigger > 0)
        {
            IntakeHatch.moveMotor(.25*mainController.rightTrigger);
        }
        else if(mainController.leftTrigger > 0)
        {
            IntakeHatch.moveMotor(-.25*mainController.leftTrigger);
        }
        else
        {
            IntakeHatch.moveMotor(0);
        }
        boolean ball = false;
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
        // AirCompressor.runCompressor();
        // mainController.vibrate(1, .9, 1);

        testDrivetrain1();
    }

    public void updateControllers()
    {
        mainController.setMainContollerValues();
        coController.setMainContollerValues();
    }
}
