package frc.team3647subsystems;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Constants;
import frc.team3647StateMachine.ArmPosition;

public class Arm extends SRXSubsystem {

    private CANSparkMax armNEO;
    private CANDigitalInput revNeoLimitSwitch;
    private CANDigitalInput fwdNeoLimitSwitch;

    private static Arm INSTANCE = new Arm();

    private Arm() {
        super(Constants.armSRXPin, Constants.armPIDF, Constants.kArmSRXCruiseVelocity, Constants.kArmSRXAcceleration,
                Constants.kArmSRXPositionThreshold);
        armNEO = new CANSparkMax(Constants.armNEOPin, CANSparkMaxLowLevel.MotorType.kBrushless);
        revNeoLimitSwitch = armNEO.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
        fwdNeoLimitSwitch = armNEO.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    }

    public static Arm getInstance() {
        return INSTANCE;
    }

    public void initSensors() {
        armNEO.restoreFactoryDefaults();
        armNEO.setIdleMode(IdleMode.kBrake);
        armNEO.enableVoltageCompensation(12);// max voltage of 12v to scale output better
        super.initSensors();
    }

    public void armNEOFollow() {
        // set to voltage that srx is output on a scale of -1 to 1
        armNEO.set(getMaster().getMotorOutputVoltage() / 12);
    }

    public void run() {
        if (!initialized)
            initSensors();

        if (aimedState != null) {
            if (!aimedState.isSpecial()) {
                setPosition(aimedState.getValue());
            } else {
                if (aimedState.equals(ArmPosition.REVLIMITSWITCH)) {
                    moveToRevLimitSwitch();
                } else if (aimedState.equals(ArmPosition.FWDLIMITSWITCH)) {
                    moveToFwdLimitSwitch();
                } else {
                    stop();
                }
            }
        }
    }

    public void moveToFwdLimitSwitch() {
        if (!getFwdLimitSwitchValue()) {
            setOpenLoop(.2);
        } else {
            stop();
        }
    }

    public void moveToRevLimitSwitch() {
        if (!getRevLimitSwitchValue()) {
            setOpenLoop(-.5);
        } else {
            resetEncoder();
            setPosition(0);
        }
    }

    @Override
    public void stop() {
        try {
            getMaster().stopMotor();
            armNEO.stopMotor();
        } catch (NullPointerException e) {
            armNEO = new CANSparkMax(Constants.armNEOPin, CANSparkMaxLowLevel.MotorType.kBrushless);
            revNeoLimitSwitch = armNEO.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
            fwdNeoLimitSwitch = armNEO.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
            initSensors();
        }
    }

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

    public void setToBrake() {
        armNEO.setIdleMode(IdleMode.kBrake);
    }

    public void setToCoast() {
        armNEO.setIdleMode(IdleMode.kCoast);
    }

    public void disableArm() {
        stop();
        setToCoast();
    }
}