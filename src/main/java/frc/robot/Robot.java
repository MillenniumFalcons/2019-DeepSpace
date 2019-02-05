package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.team3647inputs.*;
import frc.team3647pistons.IntakeHatch;
import frc.team3647subsystems.Arm;
import frc.team3647subsystems.Drivetrain;
import frc.team3647subsystems.Elevator;


public class Robot extends TimedRobot 
{

    Joysticks mainController;
    Joysticks coController;
    public static Encoders encoders;
    public static Gyro gyro;
    public static final Drivetrain drivetrain = new Drivetrain();
    public static final Elevator elevator = new Elevator();
    public static final Arm arm = new Arm();
    
 
    @Override
    public void robotInit() 
    {
        
        encoders = new Encoders();
        mainController = new Joysticks(0);
        coController = new Joysticks(1);
        gyro = new Gyro();
        
        drivetrain.leftSRX.configFactoryDefault();
        drivetrain.rightSRX.configFactoryDefault();
        drivetrain.drivetrainInitialization();
        IntakeHatch.intitialize();
        elevator.elevatorInitialization();
        TestFunctions.shuffleboard();
    }

    @Override
    public void robotPeriodic() 
    {
        TestFunctions.updateControllers(mainController, coController);
        // encoders.updateEncoderValues();
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
        IntakeHatch.hatchSRX.setSelectedSensorPosition(0);
    }

    @Override
    public void testPeriodic()
    {
        // AirCompressor.runCompressor();
        drivetrain.customArcadeDrive(.25*mainController.leftJoyStickX, .25*mainController.rightJoyStickY, gyro);
        // System.out.println(IntakeBall.reed.get());
    }

}
