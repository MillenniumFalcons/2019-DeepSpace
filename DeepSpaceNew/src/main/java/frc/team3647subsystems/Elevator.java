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

	// Sensor at bottom of elevator
	private DigitalInput limitSwitch = new DigitalInput(Constants.elevatorBeamBreakPin);


	// Elevator motors
	public VictorSPX GearboxSPX1 = new VictorSPX(Constants.ElevatorGearboxSPX1);
	public VictorSPX GearboxSPX2 = new VictorSPX(Constants.ElevatorGearboxSPX2);
	// public VictorSPX GearBoxSPX3;

	

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

		GearboxSPX2.follow(getMaster());
		GearboxSPX1.follow(getMaster());
		// GearBoxSPX3.follow(getMaster());

		GearboxSPX2.setInverted(false);
		getMaster().setInverted(false);
		GearboxSPX1.setInverted(false);
		// GearBoxSPX3.setInverted(false);

		getMaster().enableCurrentLimit(true);
		getMaster().configContinuousCurrentLimit(35);

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

	boolean reachedZeroButNotBottom = false;

	private void moveToBottom() {

		if(!getBannerSensorValue()) {
			reachedZeroButNotBottom = getEncoderValue() <= 100;
		}

		if (reachedZeroButNotBottom) {
			moveToBottomStart(.2);
			reachedZeroButNotBottom = !getBannerSensorValue();
		} else if (!getBannerSensorValue() && !reachedZeroButNotBottom) {
			setPosition(0);
		}
	}

	public void updateBannerSensor() {
		bannerSensorValue = limitSwitch.get();
	}

	public boolean getBannerSensorValue() {
		return bannerSensorValue;
	}

	public boolean isAboveMinRotate() {
		return getEncoderValue() >= Constants.elevatorMinRotation;
	}

	public boolean isAboveMinRotate(int threshold) {
		return (getEncoderValue() >= Constants.elevatorMinRotation + threshold);
	}

	public boolean isValueAboveMinRotate(int val) {
		return (val >= Constants.elevatorMinRotation - 500);
	}

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