package frc.robot;

import frc.team3647subsystems.Arm;
import frc.team3647subsystems.BallShooter;
import frc.team3647subsystems.Elevator;
import frc.team3647subsystems.HatchGrabber;
import frc.team3647subsystems.MiniShoppingCart;
import frc.team3647subsystems.SRXSubsystem;
import frc.team3647subsystems.Subsystem;

// import frc.team3647subsystems.Arm;
// import frc.team3647subsystems.BallShooter;
// import frc.team3647subsystems.Elevator;

public class TestingMethods {

    private static boolean initializedSubsystem = false;

    public static void test(Subsystem sub) {
        Robot.mainController.update();

        if (!initializedSubsystem && sub instanceof SRXSubsystem) {

            ((SRXSubsystem) sub).initSensors();
            ((SRXSubsystem) sub).setToCoast();
            initializedSubsystem = true;
        }

        if (sub instanceof Elevator) {
            testElevator((Elevator) sub);
        } else if (sub instanceof Arm) {
            testArm((Arm) sub);
        } else if (sub instanceof BallShooter) {
            testBallShooter((BallShooter) sub);
        } else if (sub instanceof HatchGrabber) {
            testHatchGrabberer((HatchGrabber) sub);
        } else if (sub instanceof MiniShoppingCart) {
            testMiniShoppingCart((MiniShoppingCart) sub);
        }
    }

    public static void reset() {
        initializedSubsystem = false;
    }

    private static void testElevator(Elevator elevInstance) {

        elevInstance.updateBannerSensor();
        elevInstance.updateEncoder();
        if (elevInstance.getBannerSensorValue()) {
            elevInstance.resetEncoder();
        }
        System.out.println("Elevator Banner Sensor: " + elevInstance.getBannerSensorValue());
        System.out.println("Elevator Encoder Value: " + elevInstance.getEncoderValue());
    }

    private static void testArm(Arm armInstance) {

        armInstance.updateEncoder();
        System.out.println("Arm Encoder Value: " + armInstance.getEncoderValue());
        System.out.println("Arm Rev Limit Switch: " + armInstance.getRevLimitSwitchValue() + "\nArm Fwd Limit Swtich: "
                + armInstance.getFwdLimitSwitchValue());
        if (armInstance.getRevLimitSwitchValue()) {
            armInstance.resetEncoder();
        }
    }

    private static void testBallShooter(BallShooter ballShooterInstance) {
        ballShooterInstance.runBlink();
        System.out.println(ballShooterInstance.cargoDetection);
    }

    private static void testHatchGrabberer(HatchGrabber hatchGrabberInstance) {
        hatchGrabberInstance.run(Robot.mainController);

        System.out.println("Hatch Grabber Current: " + HatchGrabber.getInstance().getCurrent());
    }

    private static void testMiniShoppingCart(MiniShoppingCart miniShoppingCartInstance) {
        miniShoppingCartInstance.run(Robot.mainController);
    }
}