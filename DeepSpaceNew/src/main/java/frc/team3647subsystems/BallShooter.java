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
	
	
	private CANifier canifier = new CANifier(Constants.canifierPin);
	private VictorSPX intakeMotor = new VictorSPX(Constants.ballShooterPin);
	private boolean flashed = false;
	private Timer ballBlinkTimer = new Timer();	


	public static BallShooter getInstance() {
		return INSTANCE;
	}

	public void init() {
		intakeMotor.setInverted(false);
	}

	public void runBlink() {
		if (Robot.cargoDetection) {
			if (!ballBlinkTimer.isRunning()) {
				ballBlinkTimer.reset();
				ballBlinkTimer.start();
			} else if (!flashed && ballBlinkTimer.get() > .2) {
				VisionController.limelightClimber.blink();
				VisionController.limelightFourbar.blink();
				if (ballBlinkTimer.get() > 1.2) {
					flashed = true;
				}
			}
			if (flashed) {
				VisionController.limelightClimber.setLED();
				VisionController.limelightFourbar.setLED();
			}
		} else {
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

	public void shootBall(double demand) {
		double mDemand = demand / 2;
		if (mDemand < .3) {
			mDemand = .5;
		}
		setOpenLoop(limitCurrent(mDemand));
	}

	public void intakeCargo(double power) {
		setOpenLoop(-limitCurrent(.4));
	}

	private double limitCurrent(double constant) {
		double current = Robot.pDistributionPanel.getCurrent(Constants.ballShooterPDPpin);
		return (1.3 / current + constant);
	}

	public boolean cargoDetection() {
		return !canifier.getGeneralInput(GeneralPin.LIMR);
	}

	public void printBeamBreak() {
		System.out.println("Cargo Beam Break: " + Robot.cargoDetection);
	}
}