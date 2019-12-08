package frc.team3647StateMachine;

import frc.robot.*;
import frc.team3647inputs.*;
import frc.team3647subsystems.BallIntake;
import frc.team3647subsystems.BallShooter;
import frc.team3647utility.Timer;
import frc.team3647subsystems.Elevator;
import frc.team3647subsystems.HatchGrabber;
import frc.team3647subsystems.Arm;

import frc.team3647StateMachine.Movement.*;

public class SeriesStateMachine {

    private static SeriesStateMachine INSTANCE = new SeriesStateMachine();

    /**
     * how you tell the subsystem where you want it to be
     */
    public RobotState aimedRobotState;
    public boolean initializedRobot = false;
    public boolean runClimberManually = false;
    public boolean initialized = false;

    // Variables to control initialization
    private boolean ranOnce = false;
    private int initStep = 0;

    public boolean cargoDetectedAfterPiston;

    private boolean tryingToRetractIntake = false;

    /**
     * for the coDriver a way to score hatches and get to hatch states if the cargo
     * beam break sensor is faulty, will report true
     */
    private boolean forceCargoOff = false;
    /**
     * for the coDriver, a way to force cargo on in case the ball is not detected by
     * the sensor, still able to go to ball states
     */
    private boolean forceCargoOn = false;

    // Variables to control ground cargo intake
    /**
     * was the fourbar extended and not retracted, removes delay between
     * elevator+arm going to intake and coDriver pressing left trigger
     */
    private boolean prevCargoIntakeExtended = false;
    /**
     * to keep track of how long it takes to deploy the ground intake
     */
    private Timer ballIntakeTimer = new Timer();

    // variables to control climb
    /**
     * @deprecated
     */
    private Timer climbTimer = new Timer();
    /**
     * @deprecated
     */
    private boolean ranClimbSequenceOnce = false;

    private final Movement ARRIVED = new Arrived(this::runCargoHandoff);
    private final Movement FREEMOVE = new FreeMove();
    private final Movement MOVEARM = new MoveArm();
    private final Movement MOVEELEV = new MoveElevator();
    private final Movement SAFEZMOVE = new SafeZ();
    private final Movement SAFEZDMOVE = new SafeZDown();
    private final Movement SAFEZUMOVE = new SafeZUp();

    // Variables to control movements, caching booleans to not call methods for no
    // reason

    /**
     * after checking for null, check if each subsystem reached its respective state
     * must use mSubsystem.reachedState(SubsystemAimedState) instead of
     * mSubsystem.reachedAimedState(), because we might send elevator to minRotate
     * mid sequence and it got there already, so from its point of view
     * reachedAimedState() == true, but we actually want it to go to a different one
     * specified in aimedRobotState, same for the arm.
     */
    private boolean elevatorReachedState;
    /**
     * after checking for null, check if each subsystem reached its respective state
     * must use mSubsystem.reachedState(SubsystemAimedState) instead of
     * mSubsystem.reachedAimedState(), because we might send arm to flatForwards mid
     * sequence and it got there already, so from its point of view
     * reachedAimedState() == true, but we actually want it to go to a different one
     * specified in aimedRobotState, same for the elevator.
     */
    private boolean armReachedState;
    /**
     * is elevator currently above minRotate constant, updated in void run()
     */
    private boolean elevatorAboveMinRotate;
    /**
     * is the next RobotPosition going to be above minrotation, used to help with
     * faster movement
     */
    private boolean elevatorAimedStateAboveMinRotate;

    /**
     * The min Rotate elevator level that the current arm position requires, is
     * based on the positions encoder value, for example, REVLIMITSWITCH, has a
     * higher min rotate than HATCHFLATFORWARDS
     */
    ElevatorLevel currentPositionRequiredMinRotate;
    /**
     * The min Rotate elevator level that the next arm position requires, is based
     * on the positions encoder value, for example, REVLIMITSWITCH, has a higher min
     * rotate than HATCHFLATFORWARDS
     */
    ElevatorLevel nextPositionRequiredMinRotate;
    /**
     * of the two required minRotates, next and current, the greatest value goes
     * here, the highest elevator level
     */
    int greatestMinRotateValue;
    /**
     * of the two required minRotates, next and current, the highest state goes here
     */
    ElevatorLevel greatestMinRotate;

    private boolean armHasReset = false;

    // get all subsystems from the robot
    private Arm mArm = Robot.mArm;
    private Elevator mElevator = Robot.mElevator;
    private BallIntake mBallIntake = Robot.mBallIntake;
    private BallShooter mBallShooter = Robot.mBallShooter;
    private HatchGrabber mHatchGrabber = Robot.mHatchGrabber;
    private Joysticks mainController = Robot.mainController;
    private Joysticks coController = Robot.coController;

    public static SeriesStateMachine getInstance() {
        return INSTANCE;
    }

    /**
     * initializes all the variables for robot control, only run once every game
     */
    public void init() {
        // Variables to control init
        aimedRobotState = RobotState.NONE;
        initializedRobot = false;
        initStep = 0;
        ranOnce = false;
        // aimedRobotState = RobotState.HATCHL1FORWARDS;
        // reset Variables to control cargo intake
        prevCargoIntakeExtended = false;

        initialized = true;
    }

    public void updateControllers(boolean cargoDetectionSensor, boolean isTeleop) {
        /**
         * cargo detection, after checking if hatch is extended.
         */
        boolean cargoDetection = cargoDetectionSensor && !mHatchGrabber.isExtended();
        cargoDetectedAfterPiston = cargoDetection;
        try {
            if (coController.leftJoyStickPress) {
                aimedRobotState = RobotState.VERTICALSTOWED;
            }

            if (coController.rightJoyStickPress) {
                tryingToRetractIntake = true;
                retractCargoIntake();
            }

            // starts ball intaking sequence
            if (coController.leftTrigger > .15) {
                tryingToRetractIntake = false;
                mHatchGrabber.retract(0);
                // if the robot is at the loading station state, then just run ball shooter in
                if (RobotState.CARGOLOADINGSTATIONBWD.equals(aimedRobotState)
                        || RobotState.CARGOLOADINGSTATIONFWD.equals(aimedRobotState)) {
                    mBallShooter.intakeCargo(1);
                } else {
                    // set state machine to go to cargoHandoff state from the ground intake
                    aimedRobotState = RobotState.CARGOHANDOFF;
                }
            } else {
                ballIntakeTimer.reset();
                ballIntakeTimer.stop();
                mBallIntake.stop();

                // if anything moves, run ball intake in to keep balls from flying out while
                // rotating the arm, unless the co driver is trying to shoot the ball or the hatch grabber is extended
                if (coController.rightTrigger < .15) {
                    if ((Math.abs(mArm.getEncoderVelocity()) > 700)
                            || (Math.abs(mElevator.getEncoderVelocity()) > 700) && !mHatchGrabber.isExtended()) {
                        mBallShooter.intakeCargo(.2);
                    } else {
                        mBallShooter.stop();
                    }
                }
            }

            // run ball shooter out when co driver holds right trigger,
            // The ball shooter will stop above ^
            if (coController.rightTrigger > .15) {
                mBallShooter.shootBall(coController.rightTrigger);
                mHatchGrabber.shouldPunch(!isRobotTooHighForPunch(mElevator.getEncoderValue(), aimedRobotState.getElevatorLevel().getValue()));
            } else {
                mHatchGrabber.shouldPunch(false);
            }

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
                    if (!isRobotLevel1Forwards()) {
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
                    aimedRobotState = RobotState.CARGOL1BACKWARDS;
                } else if (coController.dPadRight) {
                    aimedRobotState = RobotState.CARGOSHIPBACKWARDS;
                } else if (coController.dPadLeft) {
                    aimedRobotState = RobotState.CARGOL2BACKWARDS;
                } else if (coController.dPadUp) {
                    aimedRobotState = RobotState.CARGOL3BACKWARDS;
                }
            }

            //if we reset we have different positioning for some reason
            if(armHasReset) {
                if(aimedRobotState == RobotState.HATCHL1BACKWARDS) {
                    aimedRobotState = RobotState.HATCHL1BACKWARDSRESET;
                } else if(aimedRobotState == RobotState.HATCHL1FORWARDS) {
                    aimedRobotState = RobotState.HATCHL1FORWARDSRESET;
                }
            }


            // Main controller controls
            if (mainController.buttonA) {
                // must make aimedRobotState NONE, othewise, it will override the elevator state
                // set the next loop
                aimedRobotState = RobotState.NONE;
                mElevator.aimedState = ElevatorLevel.START;
            }

            if (mainController.buttonY) {
                armHasReset = true;
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

            if (mainController.buttonX && isTeleop) {
                aimedRobotState = RobotState.CARGOSHIPFORWARDS;
            }
        } catch (NullPointerException e) {
            aimedRobotState = RobotState.STOPPED;
        }
    }

    public void run() {

        System.out.println("Elevator Encoder: " + mElevator.getEncoderValue());
        System.out.println("Arm Encoder: " + mArm.getEncoderValue());
        // only run the sequence if the state is an actual one, having a NONE state
        // makes having a NullPointerExcpetion less likely
        try {
            if (!RobotState.NONE.equals(aimedRobotState)) {
                if (RobotState.START.equals(aimedRobotState)) {
                    initializeRobotPosition();
                } else if (RobotState.CARGOHANDOFF.equals(aimedRobotState)) {
                    cargoHandoff();
                } else if (RobotState.REVLIMITSWITCH.equals(aimedRobotState)) {
                    movementCheck(aimedRobotState).run();
                } else if (RobotState.STOPPED.equals(aimedRobotState)) {
                    mElevator.aimedState = ElevatorLevel.STOPPED;
                    mArm.aimedState = ArmPosition.STOPPED;
                } else if (!aimedRobotState.isSpecial()) {
                    goToAimedState();
                }
            }
        } catch (NullPointerException e) {
            aimedRobotState = RobotState.NONE;
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
            if (cargoDetectedAfterPiston) {
                aimedRobotState = RobotState.CARGOSHIPFORWARDS;
                //In order to automatically retract the intake after releasing
                retractCargoIntake();
            } else {
                aimedRobotState = RobotState.BEFORECARGOHANDOFF;
            }
        } else {
            mBallIntake.run();
        }
    }

    private void extendCargoGroundIntake() {
        if ((canCargoIntakeMove(mElevator) || mElevator.reachedState(RobotState.BEFORECARGOHANDOFF.getElevatorLevel())) && !tryingToRetractIntake) {
            mBallIntake.extend();
            mBallIntake.stop();
            if (!ballIntakeTimer.isRunning()) {
                ballIntakeTimer.reset();
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

    /**
     * Method will only run when robot initializes until reaching hatch level 1
     * forwards shouldn't run again
     */
    private void initializeRobotPosition() {
        if (!ranOnce) {
            switch (initStep) {
            case 0:
                movementCheck(RobotState.REVLIMITSWITCH).run();
                if (mArm.getRevLimitSwitchValue()) {
                    mArm.resetEncoder();
                    initStep = 1;
                } else if (mArm.getFwdLimitSwitchValue()) {
                    mArm.resetEncoderFwds();
                    initStep = 1;
                }
                break;
            case 1:
                setAimedRobotState(RobotState.BOTTOMSTART);
                break;
            }
        }
    }

    /**
     * goes to minrotate first then rotates the arm
     * 
     * @param pos where to rotate the arm to
     */
    private void safetyRotateArm(ArmPosition pos) {
        if (mElevator.getEncoderVelocity() > -750
                && mElevator.isAboveValue(pos.getApplicableMinRotate().getValue(), -550)
                && mElevator.getEncoderValue() <= 10000) {
            mArm.aimedState = pos;
        } else {
            mElevator.aimedState = pos.getApplicableMinRotate();
            mArm.aimedState = ArmPosition.STOPPED;
        }
    }

    /**
     * @deprecated
     */
    private void safetyRotateArmDown(ArmPosition pos) {
        mElevator.aimedState = ElevatorLevel.MINROTATE;
        mArm.aimedState = pos;
    }

    /**
     * @deprecated
     */
    private void safetyRotateArmUp(ArmPosition armPos, ElevatorLevel eLevel) {
        if (elevatorAboveMinRotate) {
            mArm.aimedState = armPos;
            mElevator.aimedState = eLevel;
        } else {
            mElevator.aimedState = ElevatorLevel.MINROTATE;
            mArm.aimedState = ArmPosition.STOPPED;
        }
    }

    /**
     * @deprecated
     */
    public int possibeArmPosition(int elevEncoder) {
        if (mArm.getEncoderValue() < Constants.armSRXVerticalStowed)
            return (int) (Math.pow((20018 - elevEncoder) / (.00022417), .5) + 13150);
        else
            return (int) (-Math.pow((20018 - elevEncoder) / (.00022417), .5) + 13150);
    }

    /**
     * @deprecated
     */
    public int possibleElevatorPosition(int armEncoder) {
        return (int) (-0.00022417 * (armEncoder - Constants.armSRXFlatForwards)
                * (armEncoder - Constants.armSRXFlatBackwards));
    }

    public boolean hasElevatorReachedAimedState() {
        return elevatorReachedState;
    }

    public boolean hasArmReachedAimedState() {
        return armReachedState;
    }

    /**
     * 
     * @param aimedState what is the next state
     * @return how and which subsystem to move next
     */
    private Movement movementCheck(RobotState aimedState) {
        Movement returnMovement;

        currentPositionRequiredMinRotate = getMinRotateBasedOnArmEncoderValue(mArm.getEncoderValue());
        nextPositionRequiredMinRotate = aimedRobotState.getArmPosition().getApplicableMinRotate();

        greatestMinRotateValue = Math.max(currentPositionRequiredMinRotate.getValue(),
                nextPositionRequiredMinRotate.getValue());

        greatestMinRotate = currentPositionRequiredMinRotate.isAboveOtherLevel(nextPositionRequiredMinRotate)
                ? currentPositionRequiredMinRotate
                : nextPositionRequiredMinRotate;
        
        if(isGoodForNoMinRotate(mArm.getEncoderValue(), aimedRobotState.getArmPosition().getValue())) {
            greatestMinRotate = ElevatorLevel.LOWESTMINROTATE;
            greatestMinRotateValue = 0;
        }
        elevatorAboveMinRotate = mElevator.isAboveValue(greatestMinRotateValue, -550);
        elevatorAimedStateAboveMinRotate = aimedRobotState.getElevatorLevel().getValue() > greatestMinRotateValue;
        armReachedState = mArm.reachedState(aimedRobotState.getArmPosition());
        elevatorReachedState = mElevator.reachedState(aimedRobotState.getElevatorLevel());

        if (elevatorReachedState && armReachedState) {
            returnMovement = ARRIVED;
        } else if (!elevatorReachedState && armReachedState) {
            // if elevator needs to move but arm is in the right state
            returnMovement = MOVEELEV;
        } else if (elevatorReachedState && !armReachedState) {
            // if arm needs to move but elevator is at right state
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

        // for use in the lambdas
        returnMovement.currentRobotState = aimedState;
        returnMovement.minRotate = greatestMinRotate;
        return returnMovement;
    }

    public RobotState getAimedRobotState() {
        return aimedRobotState;
    }

    /**
     * for the outside to tell state machine where you want it to be
     */
    public void setAimedRobotState(RobotState newAimedState) {
        aimedRobotState = newAimedState;
    }

    /**
     * retracts the intake and resets timer and varibles responsible for controlling
     * the sequence
     */
    private void retractCargoIntake() {
        mBallIntake.retract();
        prevCargoIntakeExtended = false;
        ballIntakeTimer.reset();
        ballIntakeTimer.stop();
    }

    // if elevator is at bottom or in cargo handoff, ball intake can't deploy
    private boolean canCargoIntakeMove(Elevator mElevator) {
        return mElevator.getEncoderValue() > ElevatorLevel.CARGO1.getValue() - 500
                || !mElevator.reachedState(RobotState.CARGOHANDOFF.getElevatorLevel());
    }

    private ElevatorLevel getMinRotateBasedOnArmEncoderValue(int armEncoderValue) {
        if (armEncoderValue < 4000) {
            return ElevatorLevel.MINROTATEFRONT;
        } else if (armEncoderValue > 22500) {
            return ElevatorLevel.MINROTATEBACK;
        }
        return ElevatorLevel.MINROTATE;
    }

    private boolean isGoodForNoMinRotate(int armEncoder, int futureArmEncoder) {
        boolean currentBetweenLimit = armEncoder < Constants.armSRXBackwardLimit && armEncoder > Constants.armSRXVerticalLimit;
        boolean futureBetweenLimit = futureArmEncoder < Constants.armSRXBackwardLimit && futureArmEncoder > Constants.armSRXVerticalLimit;

        return currentBetweenLimit && futureBetweenLimit;
    }

    private boolean isRobotTooHighForPunch(double currentElevatorEncoder, double aimedElevatorEncoder) {
        return currentElevatorEncoder > 35000 || aimedElevatorEncoder > 35000;
    }

    private boolean isRobotLevel1Forwards() {
        return aimedRobotState == RobotState.HATCHL1FORWARDSRESET || aimedRobotState == RobotState.HATCHL1FORWARDS;
    }
}