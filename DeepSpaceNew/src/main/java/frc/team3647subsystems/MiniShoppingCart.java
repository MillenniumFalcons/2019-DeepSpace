/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3647subsystems;

import frc.robot.*;
import frc.team3647inputs.Joysticks;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * Add your docs here.
 */
public class MiniShoppingCart extends Subsystem {
	private static MiniShoppingCart INSTANCE = new MiniShoppingCart();
	
	private WPI_TalonSRX motorSRX;

	private MiniShoppingCart() {
		motorSRX = new WPI_TalonSRX(Constants.shoppingCartMotorPin);
	}

	public static MiniShoppingCart getInstance() {
		return INSTANCE;
	}

	public void init() {
		// Motor & Sensor Direction
		motorSRX.setInverted(false);
		motorSRX.setSensorPhase(true);

		// Current Limiting
		motorSRX.configPeakCurrentLimit(Constants.kHatchWristPeakCurrent, Constants.kTimeoutMs);
		motorSRX.configPeakCurrentDuration(Constants.kHatchWristPeakCurrentDuration);
		motorSRX.configContinuousCurrentLimit(Constants.kHatchWristContinuousCurrent, Constants.kTimeoutMs);
		motorSRX.enableCurrentLimit(true);
		motorSRX.setNeutralMode(NeutralMode.Brake);
	}

	public void setOpenLoop(double demand) {
		motorSRX.set(ControlMode.PercentOutput, demand);
	}

	@Override
	public void stop() {
		motorSRX.stopMotor();
	}

	public void run(Joysticks mainController) {
		if (mainController.leftTrigger > .1) {
			setOpenLoop(mainController.leftTrigger);
		} else if (mainController.rightTrigger > .1) {
			setOpenLoop(-mainController.rightTrigger * .75);
		} else {
			stop();
		}
	}
}