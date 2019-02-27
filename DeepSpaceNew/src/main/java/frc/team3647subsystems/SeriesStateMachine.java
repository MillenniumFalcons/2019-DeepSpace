package frc.team3647subsystems;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.*;
import frc.team3647inputs.*;
import frc.team3647subsystems.Arm.ArmPosition;
import frc.team3647subsystems.Elevator.ElevatorLevel;
import frc.team3647subsystems.ShoppingCart.ShoppingCartPosition;
// import frc.team3647subsystems.ShoppingCart.WristPosition;
import frc.team3647utility.*;
import frc.team3647utility.RobotPos.Movement;

public class SeriesStateMachine
{
    public static RobotPos robotState;

    // Variables to control initialization
    private static boolean arrivedAtRevLimitSwitchOnce=false, arrivedAtFlatForwardsOnce=false, ranOnce = false;
    public static boolean initializedRobot = false;

    //control ball ground intake
    private static boolean intakeExtracted = false;
    

    // Variables to control climb
    private static boolean shoppingCartDeployed=false, mopDeploy=false, extendedIntakeOnce=false;

    //Hatch scoring positions
    private static RobotPos hatchL1Forwards, hatchL1Backwards, hatchL2Forwards, hatchL2Backwards, hatchL3Forwards, hatchL3Backwards;

    //Cargo scoring positions
    private static RobotPos cargoL1Forwards, cargoL1Backwards, cargoshipForwards, cargoshipBackwards, cargoL2Forwards, cargoL2Backwards, cargoL3Forwards, cargoL3Backwards;

    private static RobotPos climbPos;
    //handoff positions
    private static RobotPos hatchHandoff, cargoHandoff;

    //Arm limit switch positions
    public static RobotPos revLimitSwitch, fwdLimitSwitch;

    //Stowed positions
    private static RobotPos stowed, verticalStowed;

    public static RobotPos bottomStart;

    public static int state = 0;
    public static ScoringPosition aimedRobotState;

    private static Timer ballIntakeTimer;

    public enum ScoringPosition
    {
        HATCHL1FORWARDS, //ARM HIT
        HATCHL1BACKWARDS, //ARM HIT
        HATCHL2FORWARDS,
        HATCHL2BACKWARDS,
        HATCHL3FORWARDS,
        HATCHL3BACKWARDS,
        CARGOSHIPFORWARDS,
        CARGOSHIPBACKWARDS,
        CARGOL1FORWARDS,
        CARGOL1BACKWARDS,
        CARGOL2FORWARDS,
        CARGOL2BACKWARDS,
        CARGOL3FORWARDS, // Move elevator first
        CARGOL3BACKWARDS, // move elevator first
        HATCHHANDOFF, //ARM HIT
        CARGOHANDOFF, //ARM HIT, make sure cargo ground intake is deployed
        STOWED, //ARM HIT, Hatch intake is stowed
        VERTICALSTOWED, //ARM HIT
        REVLIMITSWITCH,
        FWDLIMITSWITCH,
        START,
        HATCHINTAKEGROUND,
        HATCHINTAKESCORE,
        CARGOGROUNDINTAKE, 
        HATCHINTAKESTOWED,
        BOTTOMSTART,
        CLIMB,
    }

    public static void seriesStateMachineInit()
    {
        ballIntakeTimer = new Timer();
        robotState = new RobotPos(Elevator.currentState, Arm.currentState);

        hatchL1Backwards = new RobotPos(Elevator.ElevatorLevel.BOTTOM, Arm.ArmPosition.FLATBACKWARDS);
        hatchL1Forwards = new RobotPos(Elevator.ElevatorLevel.BOTTOM, Arm.ArmPosition.FLATFORWARDS);

        hatchL2Forwards = new RobotPos(Elevator.ElevatorLevel.HATCHL2, Arm.ArmPosition.FLATFORWARDS);
        hatchL2Backwards = new RobotPos(Elevator.ElevatorLevel.HATCHL2, Arm.ArmPosition.FLATBACKWARDS);

        hatchL3Forwards = new RobotPos(Elevator.ElevatorLevel.HATCHL3, Arm.ArmPosition.FLATFORWARDS);
        hatchL3Backwards = new RobotPos(Elevator.ElevatorLevel.HATCHL3, Arm.ArmPosition.FLATBACKWARDS);

        cargoL1Forwards = new RobotPos(Elevator.ElevatorLevel.CARGO1, Arm.ArmPosition.FLATBACKWARDS);
        cargoL1Backwards = new RobotPos(Elevator.ElevatorLevel.CARGO1, Arm.ArmPosition.FLATFORWARDS);

        cargoshipForwards = new RobotPos(Elevator.ElevatorLevel.CARGOSHIP, Arm.ArmPosition.FLATBACKWARDS);
        cargoshipBackwards = new RobotPos(Elevator.ElevatorLevel.CARGOSHIP, Arm.ArmPosition.FLATFORWARDS);

        cargoL2Forwards = new RobotPos(Elevator.ElevatorLevel.CARGOL2, Arm.ArmPosition.FLATBACKWARDS);
        cargoL2Backwards = new RobotPos(Elevator.ElevatorLevel.CARGOL2, Arm.ArmPosition.FLATFORWARDS);

        cargoL3Forwards = new RobotPos(Elevator.ElevatorLevel.HATCHL3, Arm.ArmPosition.CARGOL3FRONT);
        cargoL3Backwards = new RobotPos(Elevator.ElevatorLevel.HATCHL3, Arm.ArmPosition.CARGOL3BACK);

        cargoHandoff = new RobotPos(Elevator.ElevatorLevel.CARGOHANDOFF, Arm.ArmPosition.CARGOHANDOFF);
        hatchHandoff = new RobotPos(Elevator.ElevatorLevel.HATCHHANDOFF, Arm.ArmPosition.HATCHHANDOFF);

        stowed = new RobotPos(Elevator.ElevatorLevel.STOWED, Arm.ArmPosition.STOWED);
        verticalStowed = new RobotPos(Elevator.ElevatorLevel.VERTICALSTOWED, Arm.ArmPosition.VERTICALSTOWED);

        revLimitSwitch = new RobotPos(Elevator.ElevatorLevel.MINROTATE, Arm.ArmPosition.REVLIMITSWITCH);
        fwdLimitSwitch = new RobotPos(Elevator.ElevatorLevel.MINROTATE, Arm.ArmPosition.FWDLIMITSWITCH);

        bottomStart = new RobotPos(Elevator.ElevatorLevel.START, Arm.ArmPosition.FLATFORWARDS);

        climbPos = new RobotPos(Elevator.ElevatorLevel.VERTICALSTOWED, Arm.ArmPosition.CLIMB);

        aimedRobotState = null;
        arrivedAtRevLimitSwitchOnce = false;
        initializedRobot = false;
        arrivedAtFlatForwardsOnce = false;
        ranOnce = false;
        intakeExtracted = false;

        // Variables to control climb
        shoppingCartDeployed = false;
        mopDeploy = false;
        extendedIntakeOnce = false;

        aimedRobotState = ScoringPosition.START;

    }

    public static void setControllers(Joysticks mainController, Joysticks coController)
    {
        if(!BallShooter.cargoDetection())
        {
            if(coController.buttonA)
                aimedRobotState = ScoringPosition.HATCHL1FORWARDS;
            else if(coController.buttonX)
                aimedRobotState = ScoringPosition.HATCHL2FORWARDS;
            else if(coController.buttonB)
                aimedRobotState = ScoringPosition.HATCHL2FORWARDS;
            else if(coController.buttonY)
                aimedRobotState = ScoringPosition.HATCHL3FORWARDS;
            else if(coController.dPadDown)
                aimedRobotState = ScoringPosition.HATCHL1BACKWARDS;
            else if(coController.dPadLeft)
                aimedRobotState = ScoringPosition.HATCHL2BACKWARDS;
            else if(coController.dPadRight)
                aimedRobotState = ScoringPosition.HATCHL2BACKWARDS;
            else if(coController.dPadUp)
                aimedRobotState = ScoringPosition.HATCHL3BACKWARDS;
        }
        // If the robot has a ball:
        else if(BallShooter.cargoDetection())
        {
            if(coController.buttonA)
                aimedRobotState = ScoringPosition.CARGOL1FORWARDS;
            else if(coController.buttonX)
                aimedRobotState = ScoringPosition.CARGOSHIPFORWARDS;
            else if(coController.buttonB)
                aimedRobotState = ScoringPosition.CARGOL2FORWARDS;
            else if(coController.buttonY)
                aimedRobotState = ScoringPosition.CARGOL3FORWARDS;
            else if(coController.dPadDown)
                aimedRobotState = ScoringPosition.CARGOL1BACKWARDS;
            else if(coController.dPadLeft)
                aimedRobotState = ScoringPosition.CARGOSHIPBACKWARDS;
            else if(coController.dPadRight)
                aimedRobotState = ScoringPosition.CARGOL2BACKWARDS;
            else if(coController.dPadUp)
                aimedRobotState = ScoringPosition.CARGOL3BACKWARDS;
        }


        if(coController.leftBumper)
        {
            retractCargoGroundIntake();
        }
        
        if(coController.leftTrigger > .15)
        {
            if(!arrivedAtMidPos)
            {
                ballIntakeTimer.reset();
                ballIntakeTimer.start();
                aimedRobotState = ScoringPosition.CARGOGROUNDINTAKE;
            }
            else if(prevCargoIntakeExtended || ballIntakeTimer.get() > 1)
            {
                aimedRobotState = ScoringPosition.CARGOHANDOFF;
            }
        }
        else
        {
            arrivedAtMidPos = false;
        }
        
        if(coController.rightTrigger > .15)
        {
            BallShooter.shootBall();
        }
        else
        {
            BallShooter.stopMotor();
        }

           
            
        if(coController.leftJoyStickPress)
        {
            aimedRobotState = ScoringPosition.STOWED;
        }

        if(mainController.buttonX)
        {
            aimedRobotState = ScoringPosition.CLIMB;
        }
    }
    
    public static void runSeriesStateMachine()
    {
        System.out.println("AIMED STATE: " + aimedRobotState);
        System.out.println("Current ELEVATOR STATE: " + robotState.getRobotPos().eLevel);
        System.out.println("Current ARM STATE: " + robotState.getRobotPos().armPos);
        System.out.println("arrivedAtMidPos " +  arrivedAtMidPos);
        if(Arm.currentState != null && Elevator.currentState != null)
            robotState.setRobotPos(Arm.currentState, Elevator.currentState);

        
        if(aimedRobotState != null)
        {
            switch(aimedRobotState)
            {
                case START:
                    initializeRobotPosition();
                    break;
                case BOTTOMSTART:
                    bottomStart();
                    break;
                case HATCHL1FORWARDS:
                    hatchL1Forwards();
                    break;
                case HATCHL1BACKWARDS:
                    hatchL1Backwards();
                    break;
                case HATCHL2FORWARDS:
                    hatchL2Forwards();
                    break;
                case HATCHL2BACKWARDS:
                    hatchL2Backwards();
                    break;
                case HATCHL3FORWARDS:
                    hatchL3Forwards();
                    break;
                case HATCHL3BACKWARDS:
                    hatchL3Backwards();
                    break;
                case CARGOL1FORWARDS:
                    cargoL1Forwards();
                    break;
                case CARGOL1BACKWARDS:
                    cargoL1Backwards();
                    break;
                case CARGOSHIPFORWARDS:
                    cargoshipForwards();
                    break;
                case CARGOSHIPBACKWARDS:
                    cargoshipBackwards();
                    break;
                case CARGOL2FORWARDS:
                    cargoL2Forwards();
                    break;
                case CARGOL2BACKWARDS:
                    cargoL2Backwards();
                    break;
                case CARGOL3FORWARDS:
                    cargoL3Forwards();
                    break;
                case CARGOL3BACKWARDS:
                    cargoL3Backwards();
                    break;
                case HATCHHANDOFF:
                    // hatchHandoff();
                    break;
                case CARGOHANDOFF:
                    cargoHandoff();
                    break;
                case STOWED:
                    stowed();
                    break;
                case VERTICALSTOWED:
                    verticalStowed();
                    break;
                case REVLIMITSWITCH:
                    revLimitSwitch();
                    break;
                case FWDLIMITSWITCH:
                    break;
                case HATCHINTAKESCORE:
                    groundHatchIntakeScore();
                    break;
                case CARGOGROUNDINTAKE:
                    extendCargoGroundIntake();
                    break;
                // case HATCHINTAKEGROUND:
                //     groundHatchIntakeDeploy();
                //     break;
                // case HATCHINTAKESTOWED:
                //     groundHatchIntakeStowed();
                //     break;
                case CLIMB:
                    climbing();
                    break;

            }
        }
    }

    
    private static void climbing() 
    {
        if(inThreshold(Arm.armEncoderValue, Constants.armSRXClimb, 500))
        {

            if(!extendedIntakeOnce)
            {
                BallIntake.extendIntake();
                ShoppingCart.deployShoppingCart();
            }
            else if(inThreshold(ShoppingCart.shoppingCartEncoderValue, Constants.shoppingCartDeployed, 500) && !extendedIntakeOnce)
            {
                BallIntake.retractIntake();
                extendedIntakeOnce = true;
            }
            else if(extendedIntakeOnce)
            {
                Mop.deployMop();
            }
            Elevator.aimedState = null;
            if(Robot.mainController.leftTrigger > .1)
            {
                Elevator.setOpenLoop(Robot.mainController.leftTrigger);
            }
            else if(Robot.mainController.rightTrigger > .1)
            {
                Elevator.setOpenLoop(-Robot.mainController.rightTrigger);
            }
            else
            {
                Elevator.setOpenLoop(0);
            }
        }
        else
        {
            safetyRotateArm(ArmPosition.CLIMB);
        }

        
    }

    // Hatch methods--------------------------------------------
    private static void hatchL1Forwards()
    {
        switch(robotState.movementCheck(ScoringPosition.HATCHL1FORWARDS, hatchL1Forwards))
        {
            case ARRIVED:
                System.out.println("Arrived at hatchL1Forwards");
                Arm.aimedState = hatchL1Forwards.armPos;
                Elevator.aimedState = hatchL1Forwards.eLevel;
                break;
            case MOVEELEV:
                System.out.println("Moving Elevator to: " + hatchL1Forwards.eLevel);
                Elevator.aimedState = hatchL1Forwards.eLevel;
                break;
            case MOVEARM:
                System.out.println("Moving Arm to: " + hatchL1Forwards.armPos);
                Arm.aimedState = hatchL1Forwards.armPos;
                break;
            case SAFEZMOVE:
                System.out.println("Running Safe Z");
                safetyRotateArm(hatchL1Forwards.armPos);
                break;
            case FREEMOVE:
                System.out.println("Running Freemove");
                Arm.aimedState = hatchL1Forwards.armPos;
                Elevator.aimedState = hatchL1Forwards.eLevel;
                break;
        }
    }

    private static void hatchL1Backwards()
    {
        switch(robotState.movementCheck(ScoringPosition.HATCHL1BACKWARDS, hatchL1Backwards))
        {
            case ARRIVED:
                System.out.println("Arrived at hatchL1Backwards");
                Arm.aimedState = hatchL1Backwards.armPos;
                Elevator.aimedState = hatchL1Backwards.eLevel;
                break;
            case MOVEELEV:
                System.out.println("Moving Elevator to: " + hatchL1Backwards.eLevel);
                Elevator.aimedState = hatchL1Backwards.eLevel;
                break;
            case MOVEARM:
                System.out.println("Moving Arm to: " + hatchL1Backwards.armPos);
                Arm.aimedState = hatchL1Backwards.armPos;
                break;
            case SAFEZMOVE:
                System.out.println("Running Safe Z");
                safetyRotateArm(hatchL1Backwards.armPos);
                break;
            case FREEMOVE:
                System.out.println("Running free move");
                //Arm.aimedState = hatchL1Backwards.armPos;
                //Elevator.aimedState = hatchL1Backwards.eLevel;
                break;
        }
    }

    private static void hatchL2Forwards()
    {
        switch(robotState.movementCheck(ScoringPosition.HATCHL2FORWARDS, hatchL2Forwards))
        {
            case ARRIVED:
                System.out.println("Arrived at hatchL2Forwards");
                Arm.aimedState = hatchL2Forwards.armPos;
                Elevator.aimedState = hatchL2Forwards.eLevel;
                break;
            case MOVEELEV:
                System.out.println("Moving Elevator to: " + hatchL2Forwards.eLevel);
                Elevator.aimedState = hatchL2Forwards.eLevel;
                break;
            case MOVEARM:
                System.out.println("Moving Arm to: " + hatchL2Forwards.armPos);
                Arm.aimedState = hatchL2Forwards.armPos;
                break;
            case SAFEZMOVE:
                System.out.println("Running Safe Z");
                safetyRotateArm(hatchL2Forwards.armPos);
                break;
            case FREEMOVE:
                System.out.println("Running free move");
                Arm.aimedState = hatchL2Forwards.armPos;
                Elevator.aimedState = hatchL2Forwards.eLevel;
                break;
        }
    }

    private static void hatchL2Backwards()
    {
        switch(robotState.movementCheck(ScoringPosition.HATCHL2BACKWARDS, hatchL2Backwards))
        {
            case ARRIVED:
                System.out.println("Arrived at hatchL2Backwards");
                Arm.aimedState = hatchL2Backwards.armPos;
                Elevator.aimedState = hatchL2Backwards.eLevel;
                break;
            case MOVEELEV:
                System.out.println("Moving Elevator to: " + hatchL2Backwards.eLevel);
                Elevator.aimedState = hatchL2Backwards.eLevel;
                break;
            case MOVEARM:
                System.out.println("Moving Arm to: " + hatchL2Backwards.armPos);
                Arm.aimedState = hatchL2Backwards.armPos;
                break;
            case SAFEZMOVE:
                System.out.println("Running Safe Z");
                safetyRotateArm(hatchL2Backwards.armPos);
                break;
            case FREEMOVE:
                System.out.println("Running free move");
                Arm.aimedState = hatchL2Backwards.armPos;
                Elevator.aimedState = hatchL2Backwards.eLevel;
                break;
        }
    }

    private static void hatchL3Forwards()
    {
        switch(robotState.movementCheck(ScoringPosition.HATCHL3FORWARDS, hatchL3Forwards))
        {
            case ARRIVED:
                System.out.println("Arrived at hatchL3Forwards");
                Arm.aimedState = hatchL3Forwards.armPos;
                Elevator.aimedState = hatchL3Forwards.eLevel;
                break;
            case MOVEELEV:
                System.out.println("Moving Elevator to: " + hatchL3Forwards.eLevel);
                Elevator.aimedState = hatchL3Forwards.eLevel;
                break;
            case MOVEARM:
                System.out.println("Moving Arm to: " + hatchL3Forwards.armPos);
                Arm.aimedState = hatchL3Forwards.armPos;
                break;
            case SAFEZMOVE:
                System.out.println("Running Safe Z");
                safetyRotateArm(hatchL3Forwards.armPos);
                break;
            case FREEMOVE:
                System.out.println("Running free move");
                Arm.aimedState = hatchL3Forwards.armPos;
                Elevator.aimedState = hatchL3Forwards.eLevel;
                break;
        }
    }

    private static void hatchL3Backwards()
    {
        switch(robotState.movementCheck(ScoringPosition.HATCHL3BACKWARDS, hatchL3Backwards))
        {
            case ARRIVED:
                System.out.println("Arrived at hatchL3Backwards");
                Arm.aimedState = hatchL3Backwards.armPos;
                Elevator.aimedState = hatchL3Backwards.eLevel;
                break;
            case MOVEELEV:
                System.out.println("Moving Elevator to: " + hatchL3Backwards.eLevel);
                Elevator.aimedState = hatchL3Backwards.eLevel;
                break;
            case MOVEARM:
                System.out.println("Moving Arm to: " + hatchL3Backwards.armPos);
                Arm.aimedState = hatchL3Backwards.armPos;
                break;
            case SAFEZMOVE:
                System.out.println("Running Safe Z");
                safetyRotateArm(hatchL3Backwards.armPos);
                break;
            case FREEMOVE:
                System.out.println("Running free move");
                Arm.aimedState = hatchL3Backwards.armPos;
                Elevator.aimedState = hatchL3Backwards.eLevel;
                break;
        }
    }
    //----------------------------------------------------------

    // Ball methods---------------------------------------------

    private static void cargoL1Forwards()
    {
        switch(robotState.movementCheck(ScoringPosition.CARGOL1FORWARDS, cargoL1Forwards))
        {
            case ARRIVED:
                System.out.println("Arrived at cargoshipForwards");
                Arm.aimedState = cargoL1Forwards.armPos;
                Elevator.aimedState = cargoL1Forwards.eLevel;
                break;
            case MOVEELEV:
                System.out.println("Moving Elevator to: " + cargoL1Forwards.eLevel);
                Elevator.aimedState = cargoL1Forwards.eLevel;
                break;
            case MOVEARM:
                System.out.println("Moving Arm to: " + cargoL1Forwards.armPos);
                Arm.aimedState = cargoL1Forwards.armPos;
                break;
            case SAFEZMOVE:
                System.out.println("Running Safe Z");
                safetyRotateArm(cargoL1Forwards.armPos);
                break;
            case FREEMOVE:
                System.out.println("Running free move");
                Arm.aimedState = cargoL1Forwards.armPos;
                Elevator.aimedState = cargoL1Forwards.eLevel;
                break;
        }
    }

    private static void cargoL1Backwards()
    {
        switch(robotState.movementCheck(ScoringPosition.CARGOL1BACKWARDS, cargoL1Backwards))
        {
            case ARRIVED:
                System.out.println("Arrived at cargoshipForwards");
                Arm.aimedState = cargoL1Backwards.armPos;
                Elevator.aimedState = cargoL1Backwards.eLevel;
                break;
            case MOVEELEV:
                System.out.println("Moving Elevator to: " + cargoL1Backwards.eLevel);
                Elevator.aimedState = cargoL1Backwards.eLevel;
                break;
            case MOVEARM:
                System.out.println("Moving Arm to: " + cargoL1Backwards.armPos);
                Arm.aimedState = cargoL1Backwards.armPos;
                break;
            case SAFEZMOVE:
                System.out.println("Running Safe Z");
                safetyRotateArm(cargoL1Backwards.armPos);
                break;
            case FREEMOVE:
                System.out.println("Running free move");
                Arm.aimedState = cargoL1Backwards.armPos;
                Elevator.aimedState = cargoL1Backwards.eLevel;
                break;
        }
    }
    private static void cargoshipForwards()
    {
        switch(robotState.movementCheck(ScoringPosition.CARGOSHIPFORWARDS, cargoshipForwards))
        {
            case ARRIVED:
                System.out.println("Arrived at cargoshipForwards");
                Arm.aimedState = cargoshipForwards.armPos;
                Elevator.aimedState = cargoshipForwards.eLevel;
                break;
            case MOVEELEV:
                System.out.println("Moving Elevator to: " + cargoshipForwards.eLevel);
                Elevator.aimedState = cargoshipForwards.eLevel;
                break;
            case MOVEARM:
                System.out.println("Moving Arm to: " + cargoshipForwards.armPos);
                Arm.aimedState = cargoshipForwards.armPos;
                break;
            case SAFEZMOVE:
                System.out.println("Running Safe Z");
                safetyRotateArm(cargoshipForwards.armPos);
                break;
            case FREEMOVE:
                System.out.println("Running free move");
                Arm.aimedState = cargoshipForwards.armPos;
                Elevator.aimedState = cargoshipForwards.eLevel;
                break;
        }
    }

    private static void cargoshipBackwards()
    {
        switch(robotState.movementCheck(ScoringPosition.CARGOSHIPBACKWARDS, cargoshipBackwards))
        {
            case ARRIVED:
                System.out.println("Arrived at cargoshipBackwards");
                Arm.aimedState = cargoshipBackwards.armPos;
                Elevator.aimedState = cargoshipBackwards.eLevel;
                break;
            case MOVEELEV:
                System.out.println("Moving Elevator to: " + cargoshipBackwards.eLevel);
                Elevator.aimedState = cargoshipBackwards.eLevel;
                break;
            case MOVEARM:
                System.out.println("Moving Arm to: " + cargoshipBackwards.armPos);
                Arm.aimedState = cargoshipBackwards.armPos;
                break;
            case SAFEZMOVE:
                System.out.println("Running Safe Z");
                safetyRotateArm(cargoshipBackwards.armPos);
                break;
            case FREEMOVE:
                System.out.println("Running free move");
                Arm.aimedState = cargoshipBackwards.armPos;
                Elevator.aimedState = cargoshipBackwards.eLevel;
                break;
        }
    }

    private static void cargoL2Forwards()
    {
        switch(robotState.movementCheck(ScoringPosition.CARGOL2FORWARDS, cargoL2Forwards))
        {
            case ARRIVED:
                System.out.println("Arrived at cargoL2Forwards");
                Arm.aimedState = cargoL2Forwards.armPos;
                Elevator.aimedState = cargoL2Forwards.eLevel;
                break;
            case MOVEELEV:
                System.out.println("Moving Elevator to: " + cargoL2Forwards.eLevel);
                Elevator.aimedState = cargoL2Forwards.eLevel;
                break;
            case MOVEARM:
                System.out.println("Moving Arm to: " + cargoL2Forwards.armPos);
                Arm.aimedState = cargoL2Forwards.armPos;
                break;
            case SAFEZMOVE:
                System.out.println("Running Safe Z");
                safetyRotateArm(cargoL2Forwards.armPos);
                break;
            case FREEMOVE:
                System.out.println("Running free move");
                Arm.aimedState = cargoL2Forwards.armPos;
                Elevator.aimedState = cargoL2Forwards.eLevel;
                break;
        }
    }

    private static void cargoL2Backwards()
    {
        switch(robotState.movementCheck(ScoringPosition.CARGOL2BACKWARDS, cargoL2Backwards))
        {
            case ARRIVED:
                System.out.println("Arrived at cargoL2Backwards");
                Arm.aimedState = cargoL2Backwards.armPos;
                Elevator.aimedState = cargoL2Backwards.eLevel;
                break;
            case MOVEELEV:
                System.out.println("Moving Elevator to: " + cargoL2Backwards.eLevel);
                Elevator.aimedState = cargoL2Backwards.eLevel;
                break;
            case MOVEARM:
                System.out.println("Moving Arm to: " + cargoL2Backwards.armPos);
                Arm.aimedState = cargoL2Backwards.armPos;
                break;
            case SAFEZMOVE:
                System.out.println("Running Safe Z");
                safetyRotateArm(cargoL2Backwards.armPos);
                break;
            case FREEMOVE:
                System.out.println("Running free move");
                Arm.aimedState = cargoL2Backwards.armPos;
                Elevator.aimedState = cargoL2Backwards.eLevel;
                break;
        }
    }

    private static void cargoL3Forwards()
    {
        switch(robotState.movementCheck(ScoringPosition.CARGOL3FORWARDS, cargoL3Forwards))
        {
            case ARRIVED:
                System.out.println("Arrived at cargoL3Forwards");
                Arm.aimedState = cargoL3Forwards.armPos;
                Elevator.aimedState = hatchL3Forwards.eLevel;
                break;
            case MOVEELEV:
                System.out.println("Moving Elevator to: " + hatchL3Forwards.eLevel);
                Elevator.aimedState = hatchL3Forwards.eLevel;
                break;
            case MOVEARM:
                System.out.println("Moving Arm to: " + cargoL3Forwards.armPos);
                Arm.aimedState = cargoL3Forwards.armPos;
                break;
            case SAFEZMOVE:
                System.out.println("Running Safe Z");
                safetyRotateArm(cargoL3Forwards.armPos);
                break;
            case FREEMOVE:
                System.out.println("Running free move");
                Arm.aimedState = cargoL3Forwards.armPos;
                Elevator.aimedState = hatchL3Forwards.eLevel;
                break;
        }
    }

    private static void cargoL3Backwards()
    {
        switch(robotState.movementCheck(ScoringPosition.CARGOL3FORWARDS, cargoL3Backwards))
        {
            case ARRIVED:
                System.out.println("Arrived at cargoL3Backwards");
                Arm.aimedState = cargoL3Backwards.armPos;
                Elevator.aimedState = hatchL3Forwards.eLevel;
                break;
            case MOVEELEV:
                System.out.println("Moving Elevator to: " + hatchL3Forwards.eLevel);
                Elevator.aimedState = hatchL3Forwards.eLevel;
                break;
            case MOVEARM:
                System.out.println("Moving Arm to: " + cargoL3Backwards.armPos);
                Arm.aimedState = cargoL3Backwards.armPos;
                break;
            case SAFEZMOVE:
                System.out.println("Running Safe Z");
                safetyRotateArm(cargoL3Backwards.armPos);
                break;
            case FREEMOVE:
                System.out.println("Running free move");
                Arm.aimedState = cargoL3Backwards.armPos;
                Elevator.aimedState = hatchL3Forwards.eLevel;
                break;
        }
    }
    //----------------------------------------------------------

    // private static void hatchHandoff()
    // {
    //     switch(robotState.movementCheck(ScoringPosition.HATCHHANDOFF, hatchHandoff))
    //     {
    //         case ARRIVED:
    //             System.out.println("Arrived at hatchHandoff");
    //             Arm.aimedState = hatchHandoff.armPos;
    //             Elevator.aimedState = hatchHandoff.eLevel;
    //             break;
    //         case MOVEELEV:
    //             System.out.println("Moving Elevator to: " + hatchHandoff.eLevel);
    //             Elevator.aimedState = hatchHandoff.eLevel;
    //             ShoppingCart.aimedState = WristPosition.HANDOFF;
    //             break;
    //         case MOVEARM:
    //             System.out.println("Moving Arm to: " + hatchHandoff.armPos);
    //             Arm.aimedState = hatchHandoff.armPos;
    //             break;
    //         case SAFEZMOVE:
    //             System.out.println("Running Safe Z");
    //             safetyRotateArm(hatchHandoff.armPos);
    //             break;
    //         case FREEMOVE:
    //             System.out.println("Running free move");
    //             Arm.aimedState = hatchHandoff.armPos;
    //             Elevator.aimedState = hatchHandoff.eLevel;
    //             break;
    //     }
    // }

    private static void cargoHandoff()
    {
        switch(robotState.movementCheck(ScoringPosition.CARGOHANDOFF, cargoHandoff))
        {
            case ARRIVED:
                System.out.println("Arrived at cargoHandoff");
                Arm.aimedState = cargoHandoff.armPos;
                Elevator.aimedState = cargoHandoff.eLevel;

                prevCargoIntakeExtended = true;
                BallIntake.runIntake();
                if(Robot.coController.leftTrigger < .15)
                {
                    BallIntake.stopMotor();
                    BallShooter.stopMotor();
                    aimedRobotState = ScoringPosition.CARGOSHIPFORWARDS;
                }
                break;
            case MOVEELEV:
                System.out.println("Moving Elevator to: " + cargoHandoff.eLevel);
                Elevator.aimedState = cargoHandoff.eLevel;
                break;
            case MOVEARM:
                System.out.println("Moving Arm to: " + cargoHandoff.armPos);
                Arm.aimedState = cargoHandoff.armPos;
                break;
            case SAFEZMOVE:
                System.out.println("Running Safe Z");
                safetyRotateArm(cargoHandoff.armPos);
                break;
            case FREEMOVE:
                System.out.println("Running free move");
                Arm.aimedState = cargoHandoff.armPos;
                Elevator.aimedState = cargoHandoff.eLevel;
                break;
        }
    }

    //Stowed methods--------------------------------------------
    private static void stowed()
    {
        switch(robotState.movementCheck(ScoringPosition.STOWED, stowed))
        {
            case ARRIVED:
                System.out.println("Arrived at stowed");
                Arm.aimedState = stowed.armPos;
                Elevator.aimedState = stowed.eLevel;
                break;
            case MOVEELEV:
                System.out.println("Moving Elevator to: " + stowed.eLevel);
                Elevator.aimedState = stowed.eLevel;
                break;
            case MOVEARM:
                System.out.println("Moving Arm to: " + stowed.armPos);
                Arm.aimedState = stowed.armPos;
                break;
            case SAFEZMOVE:
                System.out.println("Running Safe Z");
                safetyRotateArm(stowed.armPos);
                break;
            case FREEMOVE:
                System.out.println("Running free move");
                Arm.aimedState = stowed.armPos;
                Elevator.aimedState = stowed.eLevel;
                break;
        }
    }

    private static void verticalStowed()
    {
        switch(robotState.movementCheck(ScoringPosition.STOWED, verticalStowed))
        {
            case ARRIVED:
                System.out.println("Arrived at verticalStowed");
                Arm.aimedState = verticalStowed.armPos;
                Elevator.aimedState = verticalStowed.eLevel;
                break;
            case MOVEELEV:
                System.out.println("Moving Elevator to: " + verticalStowed.eLevel);
                Elevator.aimedState = verticalStowed.eLevel;
                break;
            case MOVEARM:
                System.out.println("Moving Arm to: " + verticalStowed.armPos);
                Arm.aimedState = verticalStowed.armPos;
                break;
            case SAFEZMOVE:
                System.out.println("Running Safe Z");
                safetyRotateArm(verticalStowed.armPos);
                break;
            case FREEMOVE:
                System.out.println("Running free move");
                Arm.aimedState = verticalStowed.armPos;
                Elevator.aimedState = verticalStowed.eLevel;
                break;
        }
    }
    //----------------------------------------------------------

    private static void revLimitSwitch()
    {
        switch(robotState.movementCheck(ScoringPosition.REVLIMITSWITCH, revLimitSwitch))
        {
            case ARRIVED:
                System.out.println("Arrived at revLimitSwitch");
                Arm.currentState = revLimitSwitch.armPos;
                Elevator.aimedState = revLimitSwitch.eLevel;
                break;
            case MOVEELEV:
                System.out.println("Moving Elevator to: " + revLimitSwitch.eLevel);
                Elevator.aimedState = revLimitSwitch.eLevel;
                break;
            case MOVEARM:
                System.out.println("Moving Arm to: " + revLimitSwitch.armPos);
                Arm.aimedState = revLimitSwitch.armPos;
                break;
            case SAFEZMOVE:
                System.out.println("Running Safe Z");
                safetyRotateArm(revLimitSwitch.armPos);
                break;
            case FREEMOVE:
                System.out.println("Running free move");
                Arm.aimedState = revLimitSwitch.armPos;
                Elevator.aimedState = revLimitSwitch.eLevel;
                break;
        }
    }

    private static void bottomStart() 
    {
        switch(robotState.movementCheck(ScoringPosition.BOTTOMSTART, bottomStart))
        {
            case ARRIVED:
                System.out.println("Arrived at revLimitSwitch");
                Arm.currentState = bottomStart.armPos;
                Elevator.aimedState = bottomStart.eLevel;
                break;
            case MOVEELEV:
                System.out.println("Moving Elevator to: " + bottomStart.eLevel);
                Elevator.aimedState = bottomStart.eLevel;
                break;
            case MOVEARM:
                System.out.println("Moving Arm to: " + bottomStart.armPos);
                Arm.aimedState = bottomStart.armPos;
                break;
            case SAFEZMOVE:
                System.out.println("Running Safe Z");
                safetyRotateArm(bottomStart.armPos);
                break;
            case FREEMOVE:
                System.out.println("Running free move");
                Arm.aimedState = bottomStart.armPos;
                Elevator.aimedState = bottomStart.eLevel;
                break;
        }
    }

    //Method will only run when robot initializes until reaching hatch level 1 forwards
    // Cannot run again
    
    public static void initializeRobotPosition()
    {
        System.out.println("arrivedAtRevLimitSwitchOnce: " + arrivedAtRevLimitSwitchOnce);
        System.out.println("arrivedAtFlatForwardsOnce: " + arrivedAtFlatForwardsOnce);
        if(!ranOnce)
        {
            if(!Arm.getRevLimitSwitch() && !arrivedAtRevLimitSwitchOnce)
            {
                safetyRotateArm(ArmPosition.REVLIMITSWITCH);
            }

            if(Arm.getRevLimitSwitch())
                    arrivedAtRevLimitSwitchOnce = true;

            if(arrivedAtRevLimitSwitchOnce && !arrivedAtFlatForwardsOnce)
            {
                aimedRobotState = ScoringPosition.BOTTOMSTART;
            }

            if(Arm.currentState == ArmPosition.FLATFORWARDS && Elevator.currentState == ElevatorLevel.BOTTOM)
                arrivedAtFlatForwardsOnce=true;

            if(arrivedAtFlatForwardsOnce && arrivedAtRevLimitSwitchOnce)
            {
                ranOnce = true;
                initializedRobot = true;
                aimedRobotState = null;
            }
}

    }

    // public static void groundHatchIntakeDeploy()
    // {
    //     switch(robotState.movementCheck(ScoringPosition.STOWED, stowed))
    //     {
    //         case ARRIVED:
    //             ShoppingCart.aimedState = WristPosition.GROUND;
    //             break;
    //         default:
    //             stowed();
    //             break;
    //     }
    // }

    public static void groundHatchIntakeScore()
    {
        switch(robotState.movementCheck(ScoringPosition.STOWED, stowed))
        {
            case ARRIVED:
                // ShoppingCart.aimedState = WristPosition.SCORE;  
                break;
            default:
                stowed();
                break;
        }            
    }

    public static void groundHatchIntakeHandoff()
    {
        switch(robotState.movementCheck(ScoringPosition.STOWED, stowed))
        {
            case ARRIVED:
                aimedRobotState = ScoringPosition.HATCHHANDOFF;  
                break;
            default:
                stowed();                
                
                break;
        }
    }

    // private static void groundHatchIntakeStowed() 
    // {
    //     switch(robotState.movementCheck(ScoringPosition.STOWED, stowed))
    //     {
    //         case ARRIVED:
    //             ShoppingCart.aimedState = WristPosition.STOWED;
    //             break;
    //         default:
    //             hatchL2Forwards();
    //             break;
    //     }
    // }

    private static boolean arrivedAtMidPos=false, prevCargoIntakeExtended=false;
    public static void extendCargoGroundIntake()
    {
        if(!arrivedAtMidPos)
        {
            hatchL2Forwards();
            if(Elevator.isAboveMinRotate(0))
                arrivedAtMidPos = true;
        }
        if(arrivedAtMidPos)
        {
            BallIntake.extendIntake();
            ballIntakeTimer.start();
        }        
    }

    private static void retractCargoGroundIntake()
    {
        if(Elevator.isAboveMinRotate(-500))
        {
            BallIntake.retractIntake();
            ballIntakeTimer.reset();
        }
        else
        {
            System.out.println("Moving elevator to " + Elevator.ElevatorLevel.MINROTATE);
            Elevator.aimedState = Elevator.ElevatorLevel.MINROTATE;
        }
    }

    private static void safetyRotateArm(Arm.ArmPosition pos)
    {

        System.out.println("Safely rotating arm to: " + pos);
        if(Elevator.elevatorEncoderValue >= Constants.elevatorMinRotation - 500)
        {
            state = 1;
            Arm.aimedState = pos;
            // Elevator.aimedState  = null;
        }
        else
        {
            System.out.println("Moving elevator to " + Elevator.ElevatorLevel.MINROTATE);
            Elevator.aimedState = Elevator.ElevatorLevel.MINROTATE;
            Arm.aimedState = ArmPosition.STOPPED;
        }
    }

    
    public static boolean inThreshold(int val,int actual, int threshold)
    {
        if((val > threshold - actual) && (val < threshold + actual))
            return true;
        return false;
    }
}