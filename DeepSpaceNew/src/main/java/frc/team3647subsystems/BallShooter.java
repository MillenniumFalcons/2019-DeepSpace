package frc.team3647subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.GeneralPin;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import frc.team3647utility.*;

import frc.robot.Constants;
import frc.robot.Robot;

public class BallShooter extends Subsystem {
	private CANifier can;
	private VictorSPX intakeMotor;

	private boolean flashed, startedTimer;
	private Timer ballBlinkTimer;

	private static BallShooter INSTANCE = new BallShooter();

	private BallShooter() {
		intakeMotor = new VictorSPX(Constants.ballShooterPin);
		ballBlinkTimer = new Timer();
		flashed = false;
		can = new CANifier(Constants.canifierPin);
		startedTimer = false;
	}

	public static BallShooter getInstance() {
		return INSTANCE;
	}

	public void init() {
		intakeMotor.setInverted(false);
	}

	public void runBlink() {
		if (Robot.cargoDetection) {
			if (!startedTimer) {
				ballBlinkTimer.reset();
				ballBlinkTimer.start();
				startedTimer = true;
			} else if (!flashed && startedTimer && ballBlinkTimer.get() > .2) {
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
			startedTimer = false;
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
		return !can.getGeneralInput(GeneralPin.LIMR);
	}

	public void printBeamBreak() {
		System.out.println("Cargo Beam Break: " + Robot.cargoDetection);
	}
}