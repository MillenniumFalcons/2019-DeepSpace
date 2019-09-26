package frc.team3647subsystems;

import frc.robot.*;
import frc.team3647inputs.Joysticks;
import frc.team3647utility.LazyVictorSPX;
import frc.team3647utility.Solenoid;
import frc.team3647utility.Timer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Notifier;

public class HatchGrabber extends Subsystem {

	private static HatchGrabber INSTANCE = new HatchGrabber();

	private VictorSPX hatchSucker = new LazyVictorSPX(Constants.shoppingCartSPXPin);
	private Solenoid hatchSolenoid = new Solenoid(Constants.mopSolenoidPin);
	private double delay = 0;
	private boolean shouldExtend = true;
	private boolean shouldRetract = false;

	private boolean punched = false;
	private boolean shouldPunch = false;

	private Notifier hatchSolenoidNotifier = new Notifier(() -> {
		if (shouldPunch && !punched) {
			hatchSolenoid.set(false);
			extended = true;
			Timer.delay(.7);
			hatchSolenoid.set(true);
			extended = false;
			punched = true;
		} else {
			if (shouldExtend) {
				Timer.delay(delay);
				hatchSolenoid.set(false);
				extended = true;
			} else if (shouldRetract) {
				Timer.delay(delay);
				hatchSolenoid.set(true);
				extended = false;
			} else {
				hatchSolenoid.set(true);
				extended = false;
			}
			punched = false;
		}
	});

	private double current = 0;

	private boolean extended;

	private HatchGrabber() {
	}

	/**
	 * Starts the notifier thread that runs the piston
	 */
	public void init() {
		hatchSolenoidNotifier.startPeriodic(.02);
	}

	public static HatchGrabber getInstance() {
		return INSTANCE;
	}

	public void run(Joysticks coController) {
		if (coController.leftBumper) {
			extend(0);
			grabHatch();
		} else if (coController.rightBumper) {
			retract(0);
			releaseHatch();
		} else if (extended) {
			runConstant();
		} else {
			stop();
		}
	}

	public void setOpenLoop(double demand) {
		hatchSucker.set(ControlMode.PercentOutput, demand);
	}

	public boolean hatchIn() {
		return current > 3.5 && current < 5.5;
	}

	private void updateCurrent() {
		current = Robot.pDistributionPanel.getCurrent(Constants.hatchGrabberPDPpin);
	}

	private double limitCurrent(double motorConst, double currentConst) {
		updateCurrent();
		return current > currentConst ? (currentConst / current) * motorConst : motorConst;

	}

	public double getCurrent() {
		updateCurrent();
		return current;
	}

	public void grabHatch() {
		hatchSucker.set(ControlMode.PercentOutput, limitCurrent(.6, 3.5));
	}

	public void releaseHatch() {
		hatchSucker.set(ControlMode.PercentOutput, -limitCurrent(1, 15));
	}

	public void extend(double delay) {
		this.delay = delay;
		shouldExtend = true;
		shouldRetract = false;
	}

	public void retract(double delay) {
		this.delay = delay;
		shouldRetract = true;
		shouldExtend = false;
	}

	public void runConstant() {
		setOpenLoop(.2);
	}

	public boolean isExtended() {
		return extended;
	}

	public void shouldPunch(boolean shouldPunch) {
		this.shouldPunch = shouldPunch;
	}

}