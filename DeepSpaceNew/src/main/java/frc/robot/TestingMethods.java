package frc.robot;

import frc.team3647subsystems.Arm;
import frc.team3647subsystems.BallShooter;
import frc.team3647subsystems.Elevator;

public class TestingMethods{
    
    public static void initTestArm(){
        Arm.initSensors();
        Arm.setToCoast();
    }

    public static void testArm(){
        Arm.updateEncoder();
        Arm.printEncoders();
        Arm.printLimitSwitches();
    }

    public static void initTestElevator(){
        Elevator.initSensors();
    }

    public static void testElevator(){
        Elevator.updateBannerSensor();
        Elevator.updateEncoder();
        Elevator.printBannerSensor();
        Elevator.printElevatorEncoders();
    }

    public static void testBeamBreakSensor(){
        BallShooter.printBeamBreak();
        BallShooter.runBlink();
    }
}