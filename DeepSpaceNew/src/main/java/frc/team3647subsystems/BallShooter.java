package frc.team3647subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.GeneralPin;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import frc.team3647utility.*;

import frc.robot.Constants;
import frc.robot.Robot;

public class BallShooter extends Subsystem {

	private static BallShooter INSTANCE = new BallShooter();

	public boolean cargoDetection = false;

	/** canifier, for getting the beam break sensor value */
	private CANifier canifier = new CANifier(Constants.canifierPin);

	/**
	 * the motor that runs the shooter in or out
	 */
	private VictorSPX intakeMotor = new LazyVictorSPX(Constants.ballShooterPin);
	private boolean flashed = false;
	private Timer ballBlinkTimer = new Timer();

	public static BallShooter getInstance() {
		return INSTANCE;
	}

	public void init() {
		intakeMotor.setInverted(false);
	}

	/**
	 * the blinker methods for everytime we sense a ball with the beam break.
	 */
	public void runBlink() {
		cargoDetection = Robot.stateMachine.cargoDetectedAfterPiston;
		if (cargoDetection) {
			if (!ballBlinkTimer.isRunning()) {
				ballBlinkTimer.reset();
				ballBlinkTimer.start();
				// wait .2sec so we don't screw with the limelights
			} else if (!flashed && ballBlinkTimer.get() > .2) {
				VisionController.limelightClimber.blink();
				VisionController.limelightFourbar.blink();
				if (ballBlinkTimer.get() > 1.2) {
					flashed = true;
				}
			}
			if (flashed) {
				// reset limelights back to original control, turn on led when using, off when
				// not.
				VisionController.limelightClimber.setLED();
				VisionController.limelightFourbar.setLED();
			}
		} else {
			// reset all vars when cargo is gone to get ready for new cargo
			ballBlinkTimer.reset();
			ballBlinkTimer.stop();
			VisionController.limelightClimber.setLED();
			VisionController.limelightFourbar.setLED();
			flashed = false;
		}
	}

	public void setOpenLoop(double demand) {
		intakeMotor.set(ControlMode.PercentOutput, demand);
	}

	/**
	 * current limitng out with openloop
	 * 
	 * @param demand power to send to the shooter motor
	 */
	public void shootBall(double demand) {
		double mDemand = demand / 2;
		if (mDemand < .3) {
			mDemand = .5;
		}
		setOpenLoop(limitCurrent(mDemand));
	}

	/**
	 * current limiting in with openloop
	 */
	public void intakeCargo(double power) {
		setOpenLoop(-limitCurrent(.4));
	}

	private double limitCurrent(double constant) {
		double current = Robot.pDistributionPanel.getCurrent(Constants.ballShooterPDPpin);
		return (1.3 / current + constant);
	}

	/**
	 * only run once per loop and cache values because it puts extra load on the can
	 * bus
	 * 
	 * @return is the beam break sensor triggered will also be true if the sensor is
	 *         disconnected
	 */
	private boolean cargoDetection() {
		return !canifier.getGeneralInput(GeneralPin.LIMR);
	}

	
	public void printBeamBreak() {
		System.out.println("Cargo Beam Break: " + cargoDetection);
	}
}