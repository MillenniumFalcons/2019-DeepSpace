package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3647inputs.Joysticks;
// import frc.team3647pistons.IntakeBall;
import frc.team3647pistons.*;
import frc.team3647subsystems.*;
import frc.team3647subsystems.Elevator.ElevatorLevel;

import frc.robot.Robot;
///TO BE MOVED SOMETIME, ELSEWHERE
public class TestFunctions
{
    static Drivetrain drivetrain = Robot.drivetrain;
    static Elevator elevator = Robot.elevator;
    static IntakeHatch intakeHatch = Robot.intakeHatch;
    static Arm arm = Robot.arm;
    static IntakeBall intakeBall = Robot.intakeBall;
    /**
     * Button A = bottom; Button X = low; Button B = middle; Button Y = High
     * @param mainController
     */
    public static void elevatorControllerMovement(Joysticks mainController)
    {
        if(mainController.buttonA)
        {
            elevator.setElevatorPosition(ElevatorLevel.BOTTOM);
        }
        else if(mainController.buttonX)
        {
            Robot.elevator.setElevatorPosition(ElevatorLevel.LOW);
        }
        else if(mainController.buttonB)
        {
            Robot.elevator.setElevatorPosition(ElevatorLevel.MIDDLE);
        }
        else if(mainController.buttonY)
        {
            Robot.elevator.setElevatorPosition(ElevatorLevel.MAX);
        }

    }


    public static void configDrivetrainPID()
    {
        drivetrain.configPID();
    }

    public static void updatePIDFMM()
    {        
        Robot.kP = SmartDashboard.getNumber("kP", 0.35);
        Robot.kI = SmartDashboard.getNumber("kI", 0);
        Robot.kD = SmartDashboard.getNumber("kD", 0.75);
        Robot.kF = SmartDashboard.getNumber("kF", 0);
        Robot.mVel = (int)SmartDashboard.getNumber("MM Velocity", 9000);
        Robot.mAccel = (int)SmartDashboard.getNumber("MM Acceleration", 30000);


        Robot.kP2 = SmartDashboard.getNumber("kP2", 0.1);
        Robot.kI2 = SmartDashboard.getNumber("kI2", 0);
        Robot.kD2 = SmartDashboard.getNumber("kD2", 0);
        Robot.kF2 = SmartDashboard.getNumber("kF2", 0);
    }

    /**
     * dPad Side = outside position, dpad Up = loading position; dPad down = inside position
     */
    public static void testHatchIntake(Joysticks mainController)
    {
        if(mainController.dPadSide)
        {
            intakeHatch.setPosition(IntakeHatch.HatchPosition.OUTSIDE);
        }
        else if(mainController.dPadUp)
        {
            intakeHatch.setPosition(IntakeHatch.HatchPosition.LOADING);
        }
        else if(mainController.dPadDown)
        {
            intakeHatch.setPosition(IntakeHatch.HatchPosition.INSIDE);
        }
        else
        {
            System.out.println("Intake Hatch Position " + intakeHatch.getCurrentState());
        }
        if(mainController.rightBumper)
        {
            intakeHatch.openIntake();
        }
        else
        {
            intakeHatch.closeIntake();
        }
    }

    public static void shuffleboard()
    {
        SmartDashboard.delete("Left SRX Velocity RPM");
        SmartDashboard.delete("Right SRX Velocity RPM");
        SmartDashboard.putNumber("kP", 0);
        SmartDashboard.putNumber("kI", 0);
        SmartDashboard.putNumber("kD", 0);
        SmartDashboard.putNumber("kF", 0);

        SmartDashboard.putNumber("kP2", 0.1);
        SmartDashboard.putNumber("kI2", 0);
        SmartDashboard.putNumber("kD2", 0);
        SmartDashboard.putNumber("kF2", 0);

        SmartDashboard.putNumber("MM Acceleration", 0);
        SmartDashboard.putNumber("MM Velocity", 0);   
    }

    public double rpmEquation(double srxVel)
    {
        return (((srxVel/4096)/.1)*60);

    }

    public static void updateControllers(Joysticks mainController, Joysticks coController)
    {
        mainController.setMainContollerValues();
        coController.setMainContollerValues();
    }

    //testing ball intake and drivetrain 01/26/2019
    /**
     * Either Controller bumper to actuate Ball Intake Pistons; Left and right trigger to actuate ball intake motors.
     * @param mainController
     */
    public static void testBallIntake(Joysticks mainController)
    {
        boolean ball = false;
        if(mainController.leftBumper)
        {
            if(ball == false)
            {
                intakeBall.openIntake();
                ball = true;
            }
            if(mainController.rightTrigger > 0)
                intakeBall.spinRoller(.75);

            else if(mainController.leftTrigger > 0)
                intakeBall.spinRoller(-.75);
                
            else
                intakeBall.spinRoller(0);
        }
        else
        {
            intakeBall.closeIntake();
            ball = false;
        }
    }
}