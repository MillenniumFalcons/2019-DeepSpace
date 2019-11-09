package frc.team3647subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Constants;
import frc.team3647StateMachine.ArmPosition;

/**
 * The arm class is what controls the arm on the robot, has the spark max motor
 * controller that follows the SRX and the way to get the arm to its aimed state
 */
public class Arm extends Subsystem{

    private static Arm INSTANCE = new Arm();
    public ArmPosition aimedState = ArmPosition.NONE;
    private CANSparkMax armNEO = new CANSparkMax(Constants.armNEOPin, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANDigitalInput revNeoLimitSwitch = armNEO.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    private CANDigitalInput fwdNeoLimitSwitch = armNEO.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    private CANEncoder neoEncoder = new CANEncoder(armNEO);
    private CANPIDController neoPID = new CANPIDController(armNEO);
    private double[] PIDArr = Constants.armNEOPIDF;
    private double cruiseVelocity = Constants.armNEOSmartMotionCruiseVelocity;
    private double acceleration = Constants.armNEOSmartMotionAcceleration;


    private int encoderValue, encoderVelocity, encoderThreshold;

    private boolean initialized = false;

    private Arm() {
            
    }

    public static Arm getInstance() {
        return INSTANCE;
    }

    public void init() {
        updateEncoder();
        initSensors();
        setEncoderValue(Constants.armSRXVerticalStowed);
    }

    /**
     * What should run before teleop, initializes the arm but doesn't reset the
     * encoder so can run midgame
     */
    public void initSensors() {
        armNEO.restoreFactoryDefaults();
        // Brake mode for easier PID
        armNEO.setIdleMode(IdleMode.kBrake);
        armNEO.enableVoltageCompensation(12);// max voltage of 12v to scale output better
        neoEncoder = new CANEncoder(armNEO);
        neoPID = new CANPIDController(armNEO);
        neoPID.setFeedbackDevice(neoEncoder);
        configPIDFMM(PIDArr[0], PIDArr[1], PIDArr[2], PIDArr[3], cruiseVelocity, acceleration);
    }

    protected void configPIDFMM(double p, double i, double d, double f, double vel, double accel) {
        neoEncoder.setPositionConversionFactor(Constants.ratioOfSrxToNeoEncoders);
        neoEncoder.setVelocityConversionFactor(Constants.ratioOfSrxToNeoEncoders * Constants.rpmToRevPer100ms);

        neoPID.setP(p, Constants.allSRXPID);
		neoPID.setI(i, Constants.allSRXPID);
		neoPID.setD(d, Constants.allSRXPID);
		neoPID.setFF(f, Constants.allSRXPID);
		// Motion Magic Constants
		neoPID.setSmartMotionMaxVelocity(vel / (Constants.ratioOfSrxToNeoEncoders * Constants.rpmToRevPer100ms), Constants.allSRXPID);
        neoPID.setSmartMotionMaxAccel(accel, Constants.allSRXPID);
    }
    

    public void run() {
        if (!initialized) {
            initSensors();
        }

        // getMaster().selectProfileSlot(slotIdx, pidIdx); 
        if (aimedState != null) {
            // None-special arm positions are those that have encoder values for the arm
            // (rotational values, degree like)

            if (!aimedState.isSpecial() && aimedState.getValue() != -1) {
                setPosition(aimedState.getValue());
            } else if (aimedState.equals(ArmPosition.REVLIMITSWITCH)) {
                moveToRevLimitSwitch();
            } else if (aimedState.equals(ArmPosition.FWDLIMITSWITCH)) {
                moveToFwdLimitSwitch();
            } else {
                stop();
            }
        }
    }

    /**
     * openloop movement to forward limit switch
     */
    public void moveToFwdLimitSwitch() {
        if (!getFwdLimitSwitchValue()) {
            // System.out.println("Moving to fwd limit switch");
            setOpenLoop(.2);
        } else {
            stop();
        }
    }

    /**
     * openloop movement to reverse limit switch
     */
    public void moveToRevLimitSwitch() {
        if (!getRevLimitSwitchValue()) {
            setOpenLoop(-.3);
        } else {
            resetEncoder();
            setPosition(0);
        }
    }

    /**
     * resets encoder value with the forwards encoder contstant, in case the open
     * loop backwards actually move forwards
     */
    public void resetEncoderFwds() {
        setEncoderValue(Constants.armSRXFwdLimitSwitch);
    }

    public void setOpenLoop(double power) {
        armNEO.set(power);
    }
    /**
     * overrides from inherited stop to stop the neo and the SRX
     */
    public void stop() {
        try {
            setOpenLoop(0);
            armNEO.stopMotor();
        } catch (NullPointerException e) {
            armNEO = new CANSparkMax(Constants.armNEOPin, CANSparkMaxLowLevel.MotorType.kBrushless);
            revNeoLimitSwitch = armNEO.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
            fwdNeoLimitSwitch = armNEO.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
            initSensors();
        }
    }

    /**
     * pulls limitswitch value from the neo, only run once per loop, cache value
     * 
     * @return is arm at reverse limitswitch
     */
    public boolean getRevLimitSwitchValue() {
        try {
            return revNeoLimitSwitch.get();
        } catch (NullPointerException e) {
            armNEO = new CANSparkMax(Constants.armNEOPin, CANSparkMaxLowLevel.MotorType.kBrushless);
            revNeoLimitSwitch = armNEO.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
            return revNeoLimitSwitch.get();
        }
    }

    public boolean getFwdLimitSwitchValue() {
        try {
            return fwdNeoLimitSwitch.get();
        } catch (NullPointerException e) {
            armNEO = new CANSparkMax(Constants.armNEOPin, CANSparkMaxLowLevel.MotorType.kBrushless);
            fwdNeoLimitSwitch = armNEO.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
            return fwdNeoLimitSwitch.get();
        }
    }

    public void setEncoderValue(double value) {
        neoEncoder.setPosition(value);
    }

    public void resetEncoder() {
        setEncoderValue(0);
    }


    protected boolean positionThreshold(double constant) {
		return (constant + encoderThreshold) > encoderValue && (constant - encoderThreshold) < encoderValue;
	}

	/**
	 * @return is the subsystem encoder within a threshold of the parameter state's
	 *         encoder
	 * @param nAimedState state to check against
	 */
	public boolean reachedState(ArmPosition nAimedState) {
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
    

    public void updateEncoder() {
        encoderValue = (int)neoEncoder.getPosition();
        encoderVelocity = (int)neoEncoder.getVelocity();
    }

    public void setPosition(double position) {
        neoPID.setReference(position, ControlType.kSmartMotion);
    }

    public void setToBrake() {
        armNEO.setIdleMode(IdleMode.kBrake);
    }

    public void setToCoast() {
        armNEO.setIdleMode(IdleMode.kCoast);
    }

    public String getName() {
        return "Arm";
    }

    public void disableArm() {
        stop();
        setToCoast();
    }

    public void printEncoder() {
        System.out.println("Arm encoder: " + getEncoderValue());
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
}