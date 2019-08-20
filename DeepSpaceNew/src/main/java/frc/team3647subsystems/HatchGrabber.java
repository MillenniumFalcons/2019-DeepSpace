package frc.team3647subsystems;

import frc.robot.*;
import frc.team3647inputs.Joysticks;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class HatchGrabber extends Subsystem {
	private VictorSPX hatchSucker;
	private double current;

	private static HatchGrabber INSTANCE = new HatchGrabber();

	private HatchGrabber() {
		hatchSucker = new VictorSPX(Constants.shoppingCartSPXPin);
		current = 0;
	}

	public static HatchGrabber getInstance() {
		return INSTANCE;
	}

	public void run(Joysticks coController) {
		if (coController.leftBumper)
			grabHatch();
		else if (coController.rightBumper)
			releaseHatch();
		else if (!Robot.cargoDetection)
			runConstant();
		else
			stop();
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

	public void runConstant() {
		setOpenLoop(.2);
	}

}