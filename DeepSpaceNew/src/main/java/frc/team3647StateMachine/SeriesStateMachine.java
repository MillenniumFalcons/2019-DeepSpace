package frc.team3647StateMachine;

import frc.robot.*;
import frc.team3647inputs.*;
import frc.team3647subsystems.BallIntake;
import frc.team3647subsystems.BallShooter;
import frc.team3647subsystems.MiniShoppingCart;
import frc.team3647utility.Timer;
import frc.team3647subsystems.Elevator;
import frc.team3647subsystems.Arm;

import frc.team3647StateMachine.Movement.*;

public class SeriesStateMachine {

    private static SeriesStateMachine INSTANCE = new SeriesStateMachine();

    public RobotState aimedRobotState;
    public boolean initializedRobot = false;
    public boolean runClimberManually = false;
    public boolean initialized = false;

    // Variables to control initialization
    private boolean ranOnce = false;
    private int initStep = 0;

    private boolean forceCargoOff = false;
    private boolean forceCargoOn = false;

    // Variables to control ground cargo intake
    private boolean arrivedAtMidPos = false;
    private boolean prevCargoIntakeExtended = false;
    private boolean startedBallIntakeTimer = false;
    private Timer ballIntakeTimer = new Timer();

    // variables to control climb
    private Timer climbTimer = new Timer();
    private boolean ranClimbSequenceOnce = false;

    private final Movement ARRIVED = new Arrived(() -> {
        runCargoHandoff();
    });
    private final Movement FREEMOVE = new FreeMove();
    private final Movement MOVEARM = new MoveArm();
    private final Movement MOVEELEV = new MoveElevator();
    private final Movement SAFEZMOVE = new SafeZ();
    private final Movement SAFEZDMOVE = new SafeZDown();
    private final Movement SAFEZUMOVE = new SafeZUp();

    // Variables to control movements
    private boolean elevatorReachedState;
    private boolean armReachedState;
    private boolean elevatorAboveMinRotate;
    private boolean elevatorAimedStateAboveMinRotate;

    private Arm mArm = Robot.mArm;
    private Elevator mElevator = Robot.mElevator;
    private BallIntake mBallIntake = Robot.mBallIntake;
    private BallShooter mBallShooter = Robot.mBallShooter;
    private MiniShoppingCart mShoppingCart = Robot.mMiniShoppingCart;

    private Joysticks mainController = Robot.mainController;
    private Joysticks coController = Robot.coController;

    public static SeriesStateMachine getInstance() {
        return INSTANCE;
    }

    // private enum Movement {
    // ARRIVED, MOVEELEV, MOVEARM, SAFEZMOVE, SAFEZDMOVE, SAFEZUMOVE, FREEMOVE;
    // }

    public void init() {
        // Variables to control init
        aimedRobotState = RobotState.NONE;
        initializedRobot = false;
        initStep = 0;
        ranOnce = false;
        // Init aimed state
        aimedRobotState = RobotState.START;

        // reset Variables to control cargo intake
        arrivedAtMidPos = false;
        prevCargoIntakeExtended = false;
        startedBallIntakeTimer = false;

        initialized = true;
    }

    public void updateControllers(boolean cargoDetection, boolean isTeleop) {
        try {
            if ((!cargoDetection || forceCargoOff) && !forceCargoOn) {
                // hatch states
                if (coController.buttonA) {
                    aimedRobotState = RobotState.HATCHL1FORWARDS;
                } else if (coController.buttonX) {
                    aimedRobotState = RobotState.CARGOLOADINGSTATIONFWD;
                } else if (coController.buttonB) {
                    aimedRobotState = RobotState.HATCHL2FORWARDS;
                } else if (coController.buttonY) {
                    aimedRobotState = RobotState.HATCHL3FORWARDS;
                } else if (coController.dPadDown) {
                    aimedRobotState = RobotState.HATCHL1BACKWARDS;
                } else if (coController.dPadLeft) {
                    aimedRobotState = RobotState.HATCHL2BACKWARDS;
                } else if (coController.dPadRight) {
                    aimedRobotState = RobotState.CARGOLOADINGSTATIONBWD;
                } else if (coController.dPadUp) {
                    aimedRobotState = RobotState.HATCHL3BACKWARDS;
                } else if (coController.leftBumper) {
                    if (!RobotState.HATCHL1FORWARDS.equals(aimedRobotState)) {
                        aimedRobotState = RobotState.HATCHL1BACKWARDS;
                    }
                }
            } else if ((cargoDetection || forceCargoOn) && !forceCargoOff) { // If the robot has a ball:
                if (coController.buttonA) {
                    aimedRobotState = RobotState.CARGOL1FORWARDS;
                } else if (coController.buttonX) {
                    aimedRobotState = RobotState.CARGOSHIPFORWARDS;
                } else if (coController.buttonB) {
                    aimedRobotState = RobotState.CARGOL2FORWARDS;
                } else if (coController.buttonY) {
                    aimedRobotState = RobotState.CARGOL3FORWARDS;
                } else if (coController.dPadDown) {
                    aimedRobotState = RobotState.CARGOSHIPBACKWARDS;
                    retractCargoIntake();
                } else if (coController.dPadLeft) {
                    aimedRobotState = RobotState.CARGOL1BACKWARDS;
                    retractCargoIntake();
                } else if (coController.dPadRight) {
                    aimedRobotState = RobotState.CARGOL2BACKWARDS;
                    retractCargoIntake();
                } else if (coController.dPadUp) {
                    aimedRobotState = RobotState.CARGOL3BACKWARDS;
                    retractCargoIntake();
                }
            }

            if (coController.leftJoyStickPress) {
                aimedRobotState = RobotState.STOWED;
            }

            if (coController.rightJoyStickPress) {
                retractCargoIntake();
            }

            if (coController.leftTrigger > .15) {
                if (RobotState.CARGOLOADINGSTATIONBWD.equals(aimedRobotState)
                        || RobotState.CARGOLOADINGSTATIONFWD.equals(aimedRobotState)) {
                    mBallShooter.intakeCargo(1);
                } else {
                    aimedRobotState = RobotState.CARGOHANDOFF;
                }
            } else {
                ballIntakeTimer.reset();
                ballIntakeTimer.stop();
                arrivedAtMidPos = false;
                mBallIntake.stop();

                if ((Math.abs(mArm.getEncoderVelocity()) > 500) || (Math.abs(mElevator.getEncoderVelocity()) > 500)) {
                    mBallShooter.intakeCargo(.45);
                }
            }

            if (coController.rightTrigger > .15) {
                mBallShooter.shootBall(coController.rightTrigger);
            } else {
                mBallShooter.stop();
            }

            // Main controller controls
            if (mainController.buttonA) {
                aimedRobotState = RobotState.NONE;
                mElevator.aimedState = ElevatorLevel.START;
            }
            if (mainController.buttonY) {
                aimedRobotState = RobotState.REVLIMITSWITCH;
            }

            if (coController.leftMidButton && coController.rightMidButton) {
                forceCargoOn = false;
                forceCargoOff = false;
            } else if (coController.rightMidButton) {
                forceCargoOn = true;
                forceCargoOff = false;
            } else if (coController.leftMidButton) {
                forceCargoOn = false;
                forceCargoOff = true;
            }

            if (mainController.dPadLeft) {
                aimedRobotState = RobotState.CARGOLOADINGSTATIONBWD;
            } else if (mainController.dPadRight) {
                aimedRobotState = RobotState.CARGOLOADINGSTATIONFWD;
            }

            if (mainController.buttonX && isTeleop) {
                aimedRobotState = RobotState.CARGOSHIPFORWARDS;
            }

        } catch (NullPointerException e) {
            aimedRobotState = RobotState.STOPPED;
        }
    }

    public void run() {

        elevatorAboveMinRotate = mElevator.isAboveMinRotate(-550);

        boolean aimedStatesAreNotNull = aimedRobotState.getElevatorLevel() != null
                && aimedRobotState.getArmPosition() != null;

        if (aimedStatesAreNotNull) {
            elevatorReachedState = mElevator.reachedState(aimedRobotState.getElevatorLevel());
            armReachedState = mArm.reachedState(aimedRobotState.getArmPosition());
        } else {
            elevatorReachedState = false;
            armReachedState = false;
        }
        elevatorAimedStateAboveMinRotate = aimedRobotState.getElevatorLevel() != null
                ? mElevator.isStateAboveMinRotate(aimedRobotState.getElevatorLevel())
                : false;

        if (!RobotState.NONE.equals(aimedRobotState)) {
            if (RobotState.START.equals(aimedRobotState)) {
                initializeRobotPosition();
            } else if (RobotState.CARGOHANDOFF.equals(aimedRobotState)) {
                cargoHandoff();
            } else if (RobotState.REVLIMITSWITCH.equals(aimedRobotState)) {
                safetyRotateArm(ArmPosition.REVLIMITSWITCH);
            } else if (RobotState.STOPPED.equals(aimedRobotState)) {
                mElevator.aimedState = ElevatorLevel.STOPPED;
                mArm.aimedState = ArmPosition.STOPPED;
            } else if (!aimedRobotState.isSpecial()) {
                goToAimedState();
            }
        }
    }

    private void goToAimedState() {
        goToAimedState(aimedRobotState);
    }

    private void goToAimedState(RobotState aimedState) {
        movementCheck(aimedState).run();
    }

    private void runCargoHandoff() {
        prevCargoIntakeExtended = true;
        if (this.coController.leftTrigger < .15) {
            mBallIntake.stop();
            if (Robot.cargoDetection) {
                aimedRobotState = RobotState.CARGOSHIPFORWARDS;
            } else {
                aimedRobotState = RobotState.BEFORECARGOHANDOFF;
            }
        } else {
            mBallIntake.run();
        }
    }

    private void extendCargoGroundIntake() {
        if (canCargoIntakeMove(mElevator)) {
            mBallIntake.extend();
            mBallIntake.stop();
            if (!ballIntakeTimer.isRunning()) {
                ballIntakeTimer.start();
            }
        } else {
            ballIntakeTimer.stop();
            ballIntakeTimer.reset();
        }
    }

    private void cargoHandoff() {
        if (prevCargoIntakeExtended || ballIntakeTimer.get() > .5) {
            goToAimedState(RobotState.CARGOHANDOFF);
        } else {
            goToAimedState(RobotState.BEFORECARGOHANDOFF);
            extendCargoGroundIntake();
        }
    }

    // Method will only run when robot initializes until reaching hatch level 1
    // forwards
    // Cannot run again
    private void initializeRobotPosition() {
        if (!ranOnce) {
            switch (initStep) {
            case 0:
                safetyRotateArm(ArmPosition.REVLIMITSWITCH);
                if (mArm.getRevLimitSwitchValue()) {
                    mArm.resetEncoder();
                    initStep = 1;
                } else if (mArm.getFwdLimitSwitchValue()) {
                    mArm.resetEncoderFwd();
                    initStep = 1;
                }
                break;
            case 1:
                aimedRobotState = RobotState.BOTTOMSTART;
                break;
            }
        }

    }

    private void safetyRotateArm(ArmPosition pos) {
        if (mElevator.getEncoderVelocity() > -750 && elevatorAboveMinRotate && mElevator.getEncoderValue() <= 30000) {
            mArm.aimedState = pos;
        } else {
            mElevator.aimedState = ElevatorLevel.MINROTATE;
            mArm.aimedState = ArmPosition.STOPPED;
        }
    }

    private void safetyRotateArmDown(ArmPosition pos) {
        mElevator.aimedState = ElevatorLevel.MINROTATE;
        mArm.aimedState = pos;
    }

    private void safetyRotateArmUp(ArmPosition armPos, ElevatorLevel eLevel) {
        if (elevatorAboveMinRotate) {
            mArm.aimedState = armPos;
            mElevator.aimedState = eLevel;
        } else {
            mElevator.aimedState = ElevatorLevel.MINROTATE;
            mArm.aimedState = ArmPosition.STOPPED;
        }
    }

    public int possibeArmPosition(int elevEncoder) {
        if (mArm.getEncoderValue() < Constants.armSRXVerticalStowed)
            return (int) (Math.pow((20018 - elevEncoder) / (.00022417), .5) + 13150);
        else
            return (int) (-Math.pow((20018 - elevEncoder) / (.00022417), .5) + 13150);
    }

    public int possibleElevatorPosition(int armEncoder) {
        return (int) (-0.00022417 * (armEncoder - Constants.armSRXFlatForwards)
                * (armEncoder - Constants.armSRXFlatBackwards));
    }

    private Movement movementCheck(RobotState aimedState) {
        Movement returnMovement;
        if (elevatorReachedState && armReachedState) {
            returnMovement = ARRIVED;
        } else if (!elevatorReachedState && armReachedState) {
            returnMovement = MOVEELEV;
        } else if (elevatorReachedState && !armReachedState) {
            if (elevatorAboveMinRotate) {
                returnMovement = MOVEARM;
            } else {
                returnMovement = SAFEZMOVE;
            }
        } else {
            if (elevatorAboveMinRotate && elevatorAimedStateAboveMinRotate) {
                returnMovement = FREEMOVE;
            } else if (elevatorAboveMinRotate && !elevatorAimedStateAboveMinRotate) {
                returnMovement = SAFEZDMOVE;
            } else if (elevatorAimedStateAboveMinRotate) {
                returnMovement = SAFEZUMOVE;
            } else {
                returnMovement = SAFEZMOVE;
            }
        }
        returnMovement.currentRobotState = aimedState;
        return returnMovement;
    }

    public RobotState getAimedRobotState() {
        return aimedRobotState;
    }

    public void setAimedRobotState(RobotState newAimedState) {
        aimedRobotState = newAimedState;
    }

    private void retractCargoIntake() {
        mBallIntake.retract();
        ballIntakeTimer.reset();
        ballIntakeTimer.stop();
    }

    // if elevator is at bottom or in cargo handoff, ball intake can't move
    private boolean canCargoIntakeMove(Elevator mElevator) {
        return mElevator.getEncoderValue() > ElevatorLevel.CARGO1.getValue() - 500
                || !mElevator.reachedState(RobotState.CARGOHANDOFF.getElevatorLevel());
    }
}