package frc.team3647subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

// import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.team3647StateMachine.ElevatorLevel;

public class Elevator extends SRXSubsystem {

	private static Elevator INSTANCE = new Elevator();

	private boolean bannerSensorValue = false;
	private boolean reachedZeroButNotBottom = false;

	/**
	 * Sensor at bottom of elevator to detect if we are at the bottom
	 */
	private DigitalInput limitSwitch = new DigitalInput(Constants.elevatorBeamBreakPin);

	// Elevator motors
	public VictorSPX GearboxSPX1 = new VictorSPX(Constants.ElevatorGearboxSPX1);
	public VictorSPX GearboxSPX2 = new VictorSPX(Constants.ElevatorGearboxSPX2);

	private Elevator() {
		super(Constants.ElevatorGearboxSRX, Constants.interstagePIDF, Constants.kElevatorCruiseVelocity,
				Constants.kElevatorAcceleration, Constants.kElevatorPositionThreshold);
		aimedState = ElevatorLevel.STOPPED;
		// GearBoxSPX3 = new VictorSPX(Constants.ElevatorGearboxSPX3);
		bannerSensorValue = false;
	}

	public static Elevator getInstance() {
		return INSTANCE;
	}

	@Override
	public void init() {
		aimedState = ElevatorLevel.STOPPED;
		initSensors();

		setEncoderValue(5000);
		updateEncoder();
	}

	@Override
	public void initSensors() {
		super.initSensors();
		System.out.println("initialized Elevator");
		aimedState = ElevatorLevel.STOPPED;

		// set the followers for the elevator, so the two stupid motors (victors) follow
		// the smart one (talon)
		GearboxSPX2.follow(getMaster());
		GearboxSPX1.follow(getMaster());

		GearboxSPX2.setInverted(false);
		getMaster().setInverted(false);
		GearboxSPX1.setInverted(false);
		// GearBoxSPX3.setInverted(false);

		// limit the elevator current so that we don't burn motors
		getMaster().enableCurrentLimit(true);
		getMaster().configContinuousCurrentLimit(Constants.kElevatorContinuousCurrent);

		getMaster().setNeutralMode(NeutralMode.Brake);
		updateBannerSensor();

		initialized = true;
	}

	public boolean hasInitialized() {
		return initialized;
	}

	public void run() {
		if (!initialized) {
			initSensors();
		}

		// must update banner sensor value every loop
		updateBannerSensor();

		if (aimedState != null) {
			if (!aimedState.isSpecial()) {
				setPosition(aimedState.getValue());
			} else {
				if (aimedState.equals(ElevatorLevel.BOTTOM)) {
					moveToBottom();
				} else if (aimedState.equals(ElevatorLevel.STOPPED)) {
					stop();
				} else if (aimedState.equals(ElevatorLevel.START)) {
					moveToBottomStart();
				}
			}
		}
	}

	/**
	 * moves to bottom with openloop only until hit banner sensor, than stops and resets
	 * the encoder
	 */
	private void moveToBottomStart() {
		if (getBannerSensorValue()) {
			stop();
			resetEncoder();
		} else {
			setOpenLoop(-.3);
		}
	}

	private void moveToBottomStart(double speed) {
		if (getBannerSensorValue()) {
			stop();
			resetEncoder();
		} else {
			setOpenLoop(-speed);
		}
	}

	/**
	 * special movement to bottom that uses a combination of motion magic and open
	 * loop to make sure we reach the absolute bottom
	 */
	private void moveToBottom() {

		if (!getBannerSensorValue()) {
			// if we reached motion magic bottom, but we aren't at the physical bottom
			// because banner sensor isn't triggerd but we are less than 100 encoder values
			// away
			reachedZeroButNotBottom = getEncoderValue() <= 100;
		} else {
			// System.out.println("Reset encoder and stop!");
			// if the banner sensor is triggered, we should reset the encoders and stop the
			// elevator to not burn
			stop();
			resetEncoder();
		}

		if (reachedZeroButNotBottom) {
			// slowly move down until reached banner sensor
			setOpenLoop(-.2);
			reachedZeroButNotBottom = !getBannerSensorValue();
		} else if (!getBannerSensorValue() && !reachedZeroButNotBottom) {
			// use motion magic if we know elevator is farther than 100 encoder values from
			// the bottom
			setPosition(0);
		}
	}

	/**
	 * pulls the banner sensor value from the digital input and stores in variable,
	 * must be run every loop otherwise getter will be wrong
	 */
	public void updateBannerSensor() {
		bannerSensorValue = limitSwitch.get();
	}

	/**
	 * @return the last stored value of the banner sensor
	 * @see void updateBannerSensor()
	 */
	public boolean getBannerSensorValue() {
		return bannerSensorValue;
	}

	/**
	 * @return if the elevator encoder value is above the minimum rotation constant
	 * @see void updateEncoder() from SRXSubsystem parent class
	 */
	public boolean isAboveMinRotate() {
		return getEncoderValue() >= Constants.elevatorMinRotation;
	}

	public boolean isAboveMinRotate(int threshold) {
		return (getEncoderValue() >= Constants.elevatorMinRotation + threshold);
	}

	public boolean isValueAboveMinRotate(int val) {
		return (val >= Constants.elevatorMinRotation - 500);
	}

	/**
	 * 
	 * @param state ElevatorLevel to check
	 * @return is the encoder value for the state above a constant
	 */
	public boolean isStateAboveMinRotate(ElevatorLevel state) {
		if (state != null) {
			return state.isAboveMinRotate();
		}
		return false;
	}

	public TalonSRX getMasterMotor() {
		return getMaster();
	}

	public void disableElevator() {
		aimedState = null;
		stop();
	}

	public String getName() {
		return "Elevator";
	}

}