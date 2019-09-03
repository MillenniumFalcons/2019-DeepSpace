package frc.team3647subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Constants;
import frc.team3647StateMachine.ArmPosition;

public class Arm extends SRXSubsystem {

    private static Arm INSTANCE = new Arm();

    private CANSparkMax armNEO = new CANSparkMax(Constants.armNEOPin, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANDigitalInput revNeoLimitSwitch = armNEO.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    private CANDigitalInput fwdNeoLimitSwitch = armNEO.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);

    private Arm() {
        super(Constants.armSRXPin, Constants.armPIDF, Constants.kArmSRXCruiseVelocity, Constants.kArmSRXAcceleration,
                Constants.kArmSRXPositionThreshold);

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
            setOpenLoop(-.3);
        } else {
            resetEncoder();
            setPosition(0);
        }
    }

    public void resetEncoderFwd() {
        setEncoderValue(Constants.armSRXFwdLimitSwitch);
    }

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

    public TalonSRX getMasterMotor() {
        return getMaster();
    }
}