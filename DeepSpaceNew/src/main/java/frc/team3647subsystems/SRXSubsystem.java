package frc.team3647subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants;
import frc.team3647StateMachine.SubsystemAimedState;

public abstract class SRXSubsystem extends Subsystem {

	protected boolean initialized = false;

	/**
	 * what controlls the whole subsystem, the subsystem always attempts to reach
	 * its aimed state, must call run() in periodic method
	 */
	public SubsystemAimedState aimedState;

	private int encoderError, encoderValue, encoderVelocity;
	private int masterCANID;
	private double[] PIDArr;
	private int MMVelocity, MMAcceleration;
	private int encoderThreshold;

	private TalonSRX masterSRX;

	protected SRXSubsystem(int masterCANID, double[] PIDArr, int MMVelocity, int MMAcceleration, int encoderThreshold) {
		this.masterCANID = masterCANID;
		this.PIDArr = PIDArr;
		masterSRX = new TalonSRX(masterCANID);
		this.encoderThreshold = encoderThreshold;
		this.MMAcceleration = MMAcceleration;
		this.MMVelocity = MMVelocity;
	}

	public void init() {
		initSensors();
	}

	/**
	 * initializes the mag encoder, sets motion magic constants, basically must call
	 * before attempting to run the subsystem
	 */
	public void initSensors() {
		masterSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs);
		masterSRX.setSensorPhase(true);
		masterSRX.setInverted(false);

		configPIDFMM(PIDArr[0], PIDArr[1], PIDArr[2], PIDArr[3], MMVelocity, MMAcceleration);

		masterSRX.setNeutralMode(NeutralMode.Brake);
		initialized = true;
	}

	/**
	 * configures PID and Motion magic velocity, acceleration for the subsystem
	 * 
	 * @see void initSensors()
	 */
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

	/**
	 * The runner method inherited from SRX Subsystem, has to be called in periodic
	 * methods of the robot (teleop or auto) for the arm to work
	 */
	public abstract void run();

	/**
	 * 
	 * @param position what encoder value should the subsystem attempt to reach
	 */
	public void setPosition(int position) {
		// Motion Magic
		try {
			masterSRX.set(ControlMode.MotionMagic, position);
			// System.out.println(getName() + " is now going to " + position);
		} catch (NullPointerException e) {
			masterSRX = new TalonSRX(masterCANID);
			initSensors();
			masterSRX.set(ControlMode.MotionMagic, position);
		}
	}

	protected boolean positionThreshold(double constant) {
		return (constant + encoderThreshold) > encoderValue && (constant - encoderThreshold) < encoderValue;
	}

	/**
	 * @return is the subsystem encoder within a threshold of the parameter state's
	 *         encoder
	 * @param nAimedState state to check against
	 */
	public boolean reachedState(SubsystemAimedState nAimedState) {
		if (aimedState != null && !nAimedState.isSpecial()) {
			return positionThreshold(nAimedState.getValue());
		}
		return false;
	}

	/**
	 * 
	 * @return has the subsystem reached its current aimed state
	 * @see {@link reachedState(SubsystemAimedState)}
	 */
	public boolean reachedAimedState() {
		return reachedState(aimedState);
	}

	/**
	 * @param demand the power to send to the master motor
	 */
	public void setOpenLoop(double demand) {
		try {
			masterSRX.set(ControlMode.PercentOutput, demand);
			// System.out.println(getName() + " is now open loop at demand " + demand);
		} catch (NullPointerException e) {
			masterSRX = new TalonSRX(masterCANID);
			initSensors();
		}
	}

	/**
	 * basically sets open loop to 0
	 */
	public void stop() {
		try {
			setOpenLoop(0);
		} catch (NullPointerException e) {
			masterSRX = new TalonSRX(masterCANID);
			initSensors();
		}

	}

	/**
	 * pulls encoder from the motor controller and updates instance variables, must
	 * be ran to get good encoder values from the getter method
	 */
	public void updateEncoder() {
		// When the arm rotates all the way to the back encoder resets
		// Gets encoder from SRX
		try {
			encoderValue = masterSRX.getSelectedSensorPosition(0);
			encoderVelocity = masterSRX.getSelectedSensorVelocity(0);
			encoderError = masterSRX.getClosedLoopError(0); // Gets difference between aimed encoder value and current
			// encoder value
		} catch (NullPointerException e) {
			masterSRX = new TalonSRX(masterCANID);
			initSensors();
		}
	}

	/**
	 * 
	 * @param encoderValue the new encoder value to be set, no bounds checking.
	 */
	protected void setEncoderValue(int encoderValue) {
		try {
			masterSRX.setSelectedSensorPosition(encoderValue, 0, Constants.kTimeoutMs);
		} catch (NullPointerException e) {
			masterSRX = new TalonSRX(masterCANID);
			initSensors();
		}
	}

	/**
	 * sets encoder value to 0
	 */
	public void resetEncoder() {
		setEncoderValue(0);
	}

	/**
	 * checks if encoder error (how far the current encoder is from its setpoint
	 * that was set using setPosition(int)) is less than a constant
	 * 
	 * @return is encoder error less than 80
	 */
	public boolean isEncoderInThreshold() {
		// Checks the difference between the aimed encoder value and the current encoder
		// value
		// If difference less than a constant, assume current and aimed positions are
		// the same
		if (Math.abs(encoderError) < 80)
			return true;
		return false;
	}

	/**
	 * if you want to have subsystem move freely while disabled, not used while
	 * enabled
	 * 
	 * @see void setToBrake()
	 */
	public void setToCoast() {
		masterSRX.setNeutralMode(NeutralMode.Coast);
	}

	/**
	 * easier on the PID controller, shorts motor wires
	 */
	public void setToBrake() {
		masterSRX.setNeutralMode(NeutralMode.Brake);
	}

	/**
	 * @return current encoder value
	 * @see void updateEncoder()
	 */
	public int getEncoderValue() {
		return encoderValue;
	}

	/**
	 * @return current encoder velocity
	 * @see void updateEncoder()
	 */
	public int getEncoderVelocity() {
		return encoderVelocity;
	}

	/**
	 * @return current encoder error
	 * @see void updateEncoder()
	 * @see boolean isEncoderInThreshold()
	 */
	public int getEncoderError() {
		return encoderError;
	}

	protected TalonSRX getMaster() {
		return masterSRX;
	}

	public abstract String getName();
}