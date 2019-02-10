package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.team3647inputs.*;
import frc.team3647pistons.*;
import frc.team3647subsystems.*;
import static java.lang.System.out;


public class Robot extends TimedRobot 
{

    Joysticks mainController;
    Joysticks coController;
    public static Gyro gyro;
    public static final Drivetrain drivetrain = new Drivetrain();
    public static final Elevator elevator = new Elevator();
    public static Arm arm = new Arm();
    public static IntakeHatch intakeHatch;
    public static IntakeBall intakeBall;
    
 
    @Override
    public void robotInit() 
    {
        mainController = new Joysticks(0);
        coController = new Joysticks(1);
        gyro = new Gyro();
        intakeHatch = new IntakeHatch();
        intakeBall = new IntakeBall();
       // IntakeHatch.hatchSRX.set(ControlMode.PercentOutput, 0);
        //elevator.elevatorInitialization();
        TestFunctions.shuffleboard();
    }

    @Override
    public void robotPeriodic() 
    {
        gyro.updateGyro();
        TestFunctions.updateControllers(mainController, coController);
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

    }

    @Override
    public void testPeriodic()
    {
        AirCompressor.runCompressor();
        drivetrain.customArcadeDrive(.35*mainController.rightJoyStickX, .35*mainController.leftJoyStickY, gyro);
        elevator.runElevator();
        TestFunctions.testBallIntake(mainController);
        intakeHatch.runIntake(mainController);
        System.out.println("Hatch Intake switch " + intakeHatch.getLimitSwitch());
        System.out.println("Current hatch state: " + intakeHatch.getCurrentState());
        System.out.println("Hatch Encoder " + intakeHatch.getEncoder());
    }

}
