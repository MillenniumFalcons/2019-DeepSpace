package frc.team3647subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants;
import frc.team3647StateMachine.SubsystemAimedState;

public abstract class SRXSubsystem extends Subsystem {

	private int encoderError, encoderValue, encoderVelocity;
	private int masterCANID;
	private double[] PIDArr;
	private int MMVelocity, MMAcceleration;
	private int encoderThreshold;

	private WPI_TalonSRX masterSRX;

	protected boolean initialized = false;

	public SubsystemAimedState aimedState;

	protected SRXSubsystem(int masterCANID, double[] PIDArr, int MMVelocity, int MMAcceleration, int encoderThreshold) {
		this.masterCANID = masterCANID;
		this.PIDArr = PIDArr;
		masterSRX = new WPI_TalonSRX(masterCANID);
		this.encoderThreshold = encoderThreshold;
	}

	public void init() {
		initSensors();
	}

	public void initSensors() {
		masterSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs);
		masterSRX.setSensorPhase(true);
		masterSRX.setInverted(false);
		masterSRX.setNeutralMode(NeutralMode.Brake);

		configPIDFMM(PIDArr[0], PIDArr[1], PIDArr[2], PIDArr[3], MMVelocity, MMAcceleration);
		initialized = true;
	}

	protected void configPIDFMM(double p, double i, double d, double f, int vel, int accel) {
		masterSRX.selectProfileSlot(Constants.allSRXPID, 0);
		masterSRX.config_kP(Constants.allSRXPID, p, Constants.kTimeoutMs);
		masterSRX.config_kI(Constants.allSRXPID, i, Constants.kTimeoutMs);
		masterSRX.config_kD(Constants.allSRXPID, d, Constants.kTimeoutMs);
		masterSRX.config_kF(Constants.allSRXPID, f, Constants.kTimeoutMs);
		// Motion Magic Constants
		masterSRX.configMotionCruiseVelocity(vel, Constants.kTimeoutMs);
		masterSRX.configMotionAcceleration(accel, Constants.kTimeoutMs);
	}

	public abstract void run();

	protected void setPosition(int position) {
		// Motion Magic
		try {
			masterSRX.set(ControlMode.MotionMagic, position);
		} catch (NullPointerException e) {
			masterSRX = new WPI_TalonSRX(masterCANID);
			masterSRX.set(ControlMode.MotionMagic, position);
		}
	}

	protected boolean positionThreshold(double constant) {
		return (constant + encoderThreshold) > encoderValue && (constant - encoderThreshold) < encoderValue;
	}

	public boolean reachedState(SubsystemAimedState nAimedState) {
		if (aimedState != null && !nAimedState.isSpecial())
			return positionThreshold(nAimedState.getValue());
		return false;
	}

	public boolean reachedAimedState() {
		return reachedState(aimedState);
	}

	public void setOpenLoop(double demand) {
		try {
			masterSRX.set(ControlMode.PercentOutput, demand);
		} catch (NullPointerException e) {
			masterSRX = new WPI_TalonSRX(masterCANID);
			initSensors();
		}
	}

	public void stop() {
		try {
			masterSRX.stopMotor();
		} catch (NullPointerException e) {
			masterSRX = new WPI_TalonSRX(masterCANID);
			initSensors();
		}

	}

	public void updateEncoder() {
		// When the arm rotates all the way to the back encoder resets
		// Gets encoder from SRX
		try {
			encoderValue = masterSRX.getSelectedSensorPosition(0);
			encoderVelocity = masterSRX.getSelectedSensorVelocity(0);
			encoderError = masterSRX.getClosedLoopError(0); // Gets difference between aimed encoder value and current
															// encoder value
		} catch (NullPointerException e) {
			masterSRX = new WPI_TalonSRX(masterCANID);
			initSensors();
		}
	}

	protected void setEncoderValue(int encoderValue) {
		try {
			masterSRX.setSelectedSensorPosition(encoderValue, 0, Constants.kTimeoutMs);
		} catch (NullPointerException e) {
			masterSRX = new WPI_TalonSRX(masterCANID);
			initSensors();
		}
	}

	public void resetEncoder() {
		setEncoderValue(0);
	}

	public boolean isEncoderInThreshold() {
		// Checks the difference between the aimed encoder value and the current encoder
		// value
		// If difference less than a constant, assume current and aimed positions are
		// the same
		if (Math.abs(encoderError) < 80)
			return true;
		return false;
	}

	public int getEncoderValue() {
		return encoderValue;
	}

	public int getEncoderVelocity() {
		return encoderVelocity;
	}

	public int getEncoderError() {
		return encoderError;
	}

	protected WPI_TalonSRX getMaster() {
		return masterSRX;
	}
}