
package frc.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3647inputs.*;
import frc.team3647subsystems.*;

public class Robot extends IterativeRobot 
{
   
    Joysticks mainController;
    Joysticks coController;
    public static Gyro gyro;
    public static final Drivetrain drivetrain = new Drivetrain();
    
 
    @Override
    public void robotInit() 
    {
        mainController = new Joysticks(0);
        coController = new Joysticks(1);
        gyro = new Gyro();
       // IntakeHatch.hatchSRX.set(ControlMode.PercentOutput, 0);
        //elevator.elevatorInitialization();
        //TestFunctions.shuffleboard();
    }

    @Override
    public void robotPeriodic() 
    {
        gyro.updateGyro();
        //TestFunctions.updateControllers(mainController, coController);
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
    
    /***********************************************************************************/
  
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0;
    public static int mVel = 0;
    public static int mAccel = 0;

    public static double kP2 = 0;
    public static double kI2 = 0;
    public static double kD2 = 0;
    public static double kF2 = 0;

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
        //gyro.printAngles();
        setJoysticks();
        drivetrain.customArcadeDrive(mainController.leftJoyStickY, mainController.rightJoyStickX, gyro);
        //BallIntake.runSmartBallIntake(coController.leftTrigger, coController.leftBumper);
        HatchIntake.runHatchIntakeWrist(coController);
        //HatchIntake.printPosition();
        HatchGrabber.runHatchGrabber(coController.rightBumper);
      //  Arm.moveManual(coController.rightJoyStickY);
    }

    public void setJoysticks()
    {
        mainController.setMainContollerValues();
        mainController.setDPadValues();
        coController.setMainContollerValues();
        coController.setDPadValues();
    }
}
