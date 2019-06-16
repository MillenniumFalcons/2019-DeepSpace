// package frc.robot;

// import frc.team3647subsystems.Arm;
// import frc.team3647subsystems.BallShooter;
// import frc.team3647subsystems.Elevator;

// public class TestingMethods{
    
//     public static void initTestArm(){
//         Arm.getInstance().initSensors();
//         Arm.getInstance().setToCoast();
//     }

//     public static void testArm(){
//         Arm.getInstance().updateEncoder();
//         Arm.getInstance().printEncoders();
//         Arm.getInstance().printLimitSwitches();
//     }

//     public static void initTestElevator(){
//         Elevator1.initSensors();
//     }

//     public static void testElevator(){
//         Elevator1.updateBannerSensor();
//         Elevator1.updateEncoder();
//         Elevator1.printBannerSensor();
//         Elevator1.printElevatorEncoders();
//     }

//     public static void testBeamBreakSensor(){
//         BallShooter.getInstance().printBeamBreak();
//         BallShooter.getInstance().runBlink();
//     }
// }