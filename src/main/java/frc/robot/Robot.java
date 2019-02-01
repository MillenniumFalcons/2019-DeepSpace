package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3647inputs.*;
import frc.team3647pistons.AirCompressor;
import frc.team3647pistons.IntakeBall;
import frc.team3647pistons.IntakeHatch;
import frc.team3647subsystems.Drivetrain;
import frc.team3647utility.Units;


public class Robot extends TimedRobot 
{

    Joysticks mainController;
    Joysticks coController;
    public static Encoders encoders;
    public static Gyro gyro;
    public static final Drivetrain drivetrain = new Drivetrain();

    //Variables to calculate left and right max velocities
    public static double prevLeftVelocity, prevRightVelocity, maxLeftVelocity, maxRightVelocity;

    //Variables to store and calculate left and right max acceleration
    public static double prevLeftAccel, prevRightAccel, maxLeftAccel, maxRightAccel;
    
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

        SmartDashboard.putNumber("kP", .1);
        SmartDashboard.putNumber("kI", 0);
        SmartDashboard.putNumber("kD", 0);
        SmartDashboard.putNumber("kF", 0);
        
        SmartDashboard.putNumber("kP2", .1);
        SmartDashboard.putNumber("kI2", 0);
        SmartDashboard.putNumber("kD2", 0);
        SmartDashboard.putNumber("kF2", 0);

        encoders.resetEncoders();
        // Velocity variables initialization
        prevLeftVelocity = 0;
        prevRightVelocity = 0;
        maxLeftVelocity = 0;
        maxRightVelocity = 0;

        //Acceleration variables initialization
        prevLeftAccel = 0;
        prevRightAccel = 0;
        maxLeftAccel = 0;
        maxRightAccel = 0;


        SmartDashboard.putNumber("lMaxVelocity", 0);
        SmartDashboard.putNumber("lCurrentVelocity", 0);
        SmartDashboard.putNumber("rMaxVelocity", 0);
        SmartDashboard.putNumber("rCurrentVelocity", 0);


        SmartDashboard.putNumber("lMaxAccel", 0);
        SmartDashboard.putNumber("lCurrentAccel", 0);
        SmartDashboard.putNumber("rMaxAccel", 0);
        SmartDashboard.putNumber("rCurrentAccel", 0);


        SmartDashboard.putNumber("encoderValue", 0);


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
    }
    
    /***********************************************************************************/
  
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0;

    public static double kP2 = 0;
    public static double kI2 = 0;
    public static double kD2 = 0;
    public static double kF2 = 0;

    @Override
    public void testInit() 
    {
        maxLeftAccel = 0;
        maxRightAccel = 0;

        prevLeftAccel = 0;
        prevRightAccel = 0;

        maxLeftVelocity = 0;
        maxRightVelocity = 0;

        prevLeftVelocity = 0;
        prevRightVelocity = 0;
    }
    @Override
    public void testPeriodic()
    {
        AirCompressor.runCompressor();
        updatePIDF();
        drivetrainPID();
        // testDrivetrain1();
        shuffleboard();
        intakeHatch(); 
        // drivetrain.leftSRX.configFactoryDefault();
        // drivetrain.rightSRX.configFactoryDefault();
        // drivetrain.leftSRX.set(ControlMode.PercentOutput, .35);
        // drivetrain.rightSRX.set(ControlMode.PercentOutput, .35);
        drivetrain.leftSRX.set(ControlMode.PercentOutput, 1);
        drivetrain.rightSRX.set(ControlMode.PercentOutput, 1);
        encoderToVelocity();

    }


    public static void drivetrainPID()
    {
		// Config left side PID Values
		drivetrain.leftSRX.selectProfileSlot(Constants.drivePIDIdx, 0);
		drivetrain.leftSRX.config_kF(Constants.drivePIDIdx, kF, Constants.kTimeoutMs);
		drivetrain.leftSRX.config_kP(Constants.drivePIDIdx, kP, Constants.kTimeoutMs);
		drivetrain.leftSRX.config_kI(Constants.drivePIDIdx, kI, Constants.kTimeoutMs);
		drivetrain.leftSRX.config_kD(Constants.drivePIDIdx, kD, Constants.kTimeoutMs);

		// Config right side PID Values
		drivetrain.rightSRX.selectProfileSlot(Constants.drivePIDIdx, 0);
		drivetrain.rightSRX.config_kF(Constants.drivePIDIdx, kF2, Constants.kTimeoutMs);
		drivetrain.rightSRX.config_kP(Constants.drivePIDIdx, kP2, Constants.kTimeoutMs);
		drivetrain.rightSRX.config_kI(Constants.drivePIDIdx, kI2, Constants.kTimeoutMs);
        drivetrain.rightSRX.config_kD(Constants.drivePIDIdx, kD2, Constants.kTimeoutMs);
    }

    public void updatePIDF()
    {        
        kP = SmartDashboard.getNumber("kP", 0.1);
        kI = SmartDashboard.getNumber("kP", 0);
        kD = SmartDashboard.getNumber("kP", 0);
        kF = SmartDashboard.getNumber("kP", 0);

        kP2 = SmartDashboard.getNumber("kP2", 0.1);
        kI2 = SmartDashboard.getNumber("kP2", 0);
        kD2 = SmartDashboard.getNumber("kP2", 0);
        kF2 = SmartDashboard.getNumber("kP2", 0);
    }

    public void intakeHatch()
    {
        if(mainController.buttonX)
        {
            IntakeHatch.setPosition(IntakeHatch.HatchPosition.OUTSIDE);
        }
        else if(mainController.buttonY)
        {
            IntakeHatch.setPosition(IntakeHatch.HatchPosition.LOADING);
        }
        else if(mainController.buttonB)
        {
            IntakeHatch.setPosition(IntakeHatch.HatchPosition.INSIDE);
        }
        else
        {
            System.out.println("Intake Hatch Position " + IntakeHatch.currentPosition);
        }
    }

    public void shuffleboard()
    {
        SmartDashboard.putNumber("Left SRX Velocity RPM", rpmEquation(drivetrain.leftSRX.getSelectedSensorVelocity(0)));
        SmartDashboard.putNumber("Right SRX Velocity RPM", rpmEquation(drivetrain.rightSRX.getSelectedSensorVelocity(0)));
    }

    public double rpmEquation(double srxVel)
    {
        return (((srxVel/4096)/.1)*60);

    }

    public void updateControllers()
    {
        mainController.setMainContollerValues();
        coController.setMainContollerValues();
    }

    //testing ball intake and drivetrain 01/26/2019
    public void testDrivetrain1()
    {
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

    //Wheel radius in inches
    //encoder value in ticks / 100ms
    public double encoderToMpS(double encoderVal, double wheelRadius)
    {
        SmartDashboard.putNumber("encoderValue", encoderVal);
        return Math.abs(encoderVal) / .1  / 4096.0 * .0254 * wheelRadius * Math.PI * 2;
    }

    public void encoderToVelocity()
    {
        // Takes sensors encoder ticks per .1second and wheel radius to get velocity for left and right motors
        double leftMpS = encoderToMpS(drivetrain.leftSRX.getSelectedSensorVelocity(0), 3);
        double rightMpS = encoderToMpS(drivetrain.rightSRX.getSelectedSensorVelocity(0), 3);
        
        // Compare current velocity to max velocity recoreded
        if(leftMpS > maxLeftVelocity)
        {
            // If current left velocity is bigger than max, set max to current
            maxLeftVelocity = leftMpS;
        }
        if(rightMpS > maxRightVelocity)
        {
            // Same for right motor
            maxRightVelocity = rightMpS;
        }

        // Get current acceleration for left and right
        double leftAccel = (leftMpS - prevLeftVelocity) / .02;
        double rightAccel = (rightMpS - prevRightVelocity) / .02;

        // Compare previous acceleration to current acceleration
        if(leftAccel > maxLeftAccel)
        {
            maxLeftAccel = leftAccel;
        }
        if(rightAccel > maxRightAccel)
        {
            maxRightAccel = rightAccel;
        }

        // Displays current and max velocity values on smartdashboard
        SmartDashboard.putNumber("lMaxVelocity", maxLeftVelocity);
        SmartDashboard.putNumber("lCurrentVelocity", leftMpS);
        SmartDashboard.putNumber("rMaxVelocity", maxRightVelocity);
        SmartDashboard.putNumber("rCurrentVelocity", rightMpS);



        // Displays current and max acceleration values on smartdashboard
        SmartDashboard.putNumber("lMaxAccel", maxLeftAccel);
        SmartDashboard.putNumber("lCurrentAccel", leftAccel);
        SmartDashboard.putNumber("rMaxAccel", maxRightAccel);
        SmartDashboard.putNumber("rCurrentAccel", rightAccel);

        // Sets previous velocity to current velocity before method ends
        prevLeftVelocity = leftMpS;
        prevRightVelocity = rightMpS;

        // Sets previous acceleraion to current acceleraion before method ends
        prevLeftAccel = leftAccel;
        prevRightAccel = rightAccel;
    }
}
