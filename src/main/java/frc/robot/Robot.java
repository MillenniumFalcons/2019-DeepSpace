package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3647autonomous.AutoSelector;
import frc.team3647autonomous.AutoSelector.AutoMode;
import frc.team3647autonomous.AutoSelector.StartPosition;
import frc.team3647inputs.*;
import frc.team3647pistons.*;
import frc.team3647subsystems.*;
import frc.team3647utility.Odometry;

import static java.lang.System.out;


public class Robot extends TimedRobot 
{

    //RAMSETE AUTO
    Command mAutonomous;		
    SendableChooser<AutoMode> mAutoModeChooser;			//Create object to send AutoMode data to SmartDashboard
    SendableChooser<StartPosition> mStartPosChooser;
    Odometry odo;
    boolean lastAuto;


    Joysticks mainController;
    Joysticks coController;
    public static Gyro gyro;
    public static final Drivetrain drivetrain = new Drivetrain();
    public static Elevator elevator;
    public static Arm arm;
    public static IntakeHatch intakeHatch;
    public static IntakeBall intakeBall;
    
 
    @Override
    public void robotInit() 
    {                
        mainController = new Joysticks(0);
        coController = new Joysticks(1);
        //gyro = new Gyro();
        //intakeHatch = new IntakeHatch();
        // intakeBall = new IntakeBall();
        // TestFunctions.shuffleboard();
        arm = new Arm();
        //elevator = new Elevator();    
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
        //RAMSETE AUTO STUFF
        odo = new Odometry(); 	//Initialize Odometry Object (see Odometry.java)
        lastAuto = false; 	    //lastauto determines the coasting of motors

        mAutoModeChooser = new SendableChooser<>(); 								//Create a SendableChooser to put in dashboard for choosing auto
        mAutoModeChooser.setDefaultOption("TEST_PATH", AutoMode.TEST_PATH);			//Default object set to, from AutoSelector.java, AutoMode Enum: "TEST_PATH" 
        mAutoModeChooser.addOption("OPEN_LOOP_DRIVE", AutoMode.OPEN_LOOP_DRIVE);	//Adding another object from AutoSelector.java, AutoMode Enum: "OPEN_LOOP_DRIVE"
        SmartDashboard.putData("Auto Mode", mAutoModeChooser);                      //Put the data from mA.M.C. in the SmartDashboard under "Auto Mode"
        
        mStartPosChooser = new SendableChooser<>();						    //Create a SendableChooser to put in dashboard for choosing starting position of robot
        mStartPosChooser.setDefaultOption("Right", StartPosition.RIGHT);	//Default object set to, from AutoSelector.java, StartPosition Enum: "RIGHT" 
        mStartPosChooser.addOption("Middle", StartPosition.MIDDLE);		    //Adding another object from AutoSelector.java, StartPosition Enum: "MIDDLE"
        mStartPosChooser.addOption("Left", StartPosition.LEFT);			    //Adding another object from AutoSelector.java, StartPosition Enum: "LEFT"
        SmartDashboard.putData("Start Position", mStartPosChooser);		    //Put the data from mS.P.C. in the SmartDashboard under "Start Position"
    
        resetForAuto();
		mAutonomous = new AutoSelector		//Create new AutoSelector Object to select auto behavior for the robot to run
		(
			mStartPosChooser.getSelected(), //Gets robot start position from SendableChooser on Dashboard
			mAutoModeChooser.getSelected()	//Gets robot Auto Mode from SendableChooser on Dashboard
		);
		mAutonomous.start();
    }
    
    private void resetForAuto() 
    {
        
		odo.setOdometry(0, 0, 0); // Resets Odometry Values
		drivetrain.resetEncoders(); // Reset Encoder Values to zero
		gyro.resetAngle(); // Resets Gyro Yaw Angle to zero
    }

    @Override
    public void autonomousPeriodic() 
    {
        Scheduler.getInstance().run();	//Runs the method once
		lastAuto = true;				//Sets lastAuto to true, thus setting brakes to coast after 
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
        //AirCompressor.runCompressor();
        // drivetrain.customArcadeDrive(.35*mainController.rightJoyStickX, .35*mainController.leftJoyStickY, gyro);
        // elevator.runElevator();
        // TestFunctions.testBallIntake(mainController);
        // intakeHatch.runIntake(mainController);
        //elevator.moveElevator(mainController.leftJoyStickY);
        arm.moveArm(.5*mainController.rightJoyStickX);
        // System.out.println("Hatch Intake switch " + intakeHatch.getLimitSwitch());
        // System.out.println("Current hatch state: " + intakeHatch.getCurrentState());
        // System.out.println("Hatch Encoder " + intakeHatch.getEncoder());
        out.println("Arm Encoder Value: " + arm.getEncoder());
        out.println("Elevator Encoder Value " + elevator.getEncoder());

        if(mainController.leftBumper)
        {
            arm.resetEncoder();
            elevator.resetEncoder();
        }
    }

}
