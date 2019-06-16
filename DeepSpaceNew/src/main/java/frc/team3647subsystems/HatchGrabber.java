package frc.team3647subsystems;

import frc.robot.*;
import frc.team3647inputs.Joysticks;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class HatchGrabber extends Subsystem {
	private VictorSPX hatchSucker;

	private static HatchGrabber INSTANCE = new HatchGrabber();

	private HatchGrabber() {
		hatchSucker = new VictorSPX(Constants.shoppingCartSPXPin);
	}

	public static HatchGrabber getInstance() {
		return INSTANCE;
	}

	public void run(Joysticks coController) {
		if (coController.rightBumper)
			grabHatch();
		else if (coController.leftBumper)
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
		double current = Robot.pDistributionPanel.getCurrent(Constants.hatchGrabberPDPpin);
		return current > 3.5 && current < 5.5;
	}

	public void grabHatch() {
		setOpenLoop(.6);
	}

	public void releaseHatch() {
		setOpenLoop(-1);
	}

	public void runConstant() {
		setOpenLoop(.2);
	}

}