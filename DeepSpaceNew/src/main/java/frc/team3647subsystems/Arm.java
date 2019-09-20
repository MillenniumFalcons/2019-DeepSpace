package frc.team3647subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Constants;
import frc.team3647StateMachine.ArmPosition;

/**
 * The arm class is what controls the arm on the robot, has the spark max motor
 * controller that follows the SRX and the way to get the arm to its aimed state
 */
public class Arm extends SRXSubsystem {

    private static Arm INSTANCE = new Arm();

    private CANSparkMax armNEO = new CANSparkMax(Constants.armNEOPin, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANDigitalInput revNeoLimitSwitch = armNEO.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    private CANDigitalInput fwdNeoLimitSwitch = armNEO.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    private CANEncoder neoEncoder = new CANEncoder(armNEO);

    private Arm() {
        super(Constants.armSRXPin, Constants.armPIDF, Constants.kArmSRXCruiseVelocity, Constants.kArmSRXAcceleration,
                Constants.kArmSRXPositionThreshold);

    }

    public static Arm getInstance() {
        return INSTANCE;
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
        super.initSensors();
    }


    /**
     * follower code for the NEO because it wouldn't follow a talon with motion
     * magic otherwise
     */
    public void armNEOFollow() {
        // set to voltage that srx is output on a scale of -1 to 1
        armNEO.set(getMaster().getMotorOutputVoltage() / 12);
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

    /**
     * overrides from inherited stop to stop the neo and the SRX
     */
    @Override
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
     * move NEO manually to make sure it works, shouldn't be used normally
     */
    public void moveNEO(double power) {
        armNEO.set(power);
        System.out.println(armNEO.getMotorTemperature());
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

    @Override
    public void setToBrake() {
        armNEO.setIdleMode(IdleMode.kBrake);
    }

    @Override
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

    public TalonSRX getMasterMotor() {
        return getMaster();
    }

    public double getNEOEncoderValue() {
        return neoEncoder.getPosition();
    }
}