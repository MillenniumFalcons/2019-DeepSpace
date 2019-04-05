package frc.team3647subsystems;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.*;
import frc.team3647inputs.*;
import frc.team3647subsystems.Arm.ArmPosition;
import frc.team3647subsystems.Elevator.ElevatorLevel;

public class SeriesStateMachine
{
    // Variables to control initialization
    private static boolean ranOnce = false;
    public static boolean initializedRobot = false;
    private static short initStep = 1;

    // Variables to control climb
    public static boolean shoppingCartDeployed=false, mopDeploy=false, extendedIntakeOnce=false, elevatorManual = false, climbMode=false;
    private static short climbStep = 0;

    public static ScoringPosition aimedRobotState;

    public static boolean forceCargoOff = false;
    public static boolean forceCargoOn = false;

    //Variables to control ground cargo intake
    private static boolean arrivedAtMidPos = false, prevCargoIntakeExtended = false,  startedBallIntakeTimer = false;;

    private static Timer ballIntakeTimer = new Timer(), climbTimer = new Timer();

    private static boolean elevatorReachedState, armReachedState, elevatorAboveMinRotate, elevatorAimedStateAboveMinRotate;


    public enum ScoringPosition
    {
        HATCHL1FORWARDS(Elevator.ElevatorLevel.BOTTOM, Arm.ArmPosition.FLATFORWARDS), //ARM HIT
        HATCHL1BACKWARDS(Elevator.ElevatorLevel.BOTTOM, Arm.ArmPosition.FLATBACKWARDS), //ARM HIT
        HATCHL1VISIONFORWARDS(Elevator.ElevatorLevel.BOTTOM, Arm.ArmPosition.FLATVISIONFORWARDS),
        HATCHL1VISIONBACKWARDS(Elevator.ElevatorLevel.BOTTOM, Arm.ArmPosition.FLATVISIONBACKWARDS),
        HATCHL2FORWARDS(Elevator.ElevatorLevel.HATCHL2, Arm.ArmPosition.FLATFORWARDS),
        HATCHL2BACKWARDS(Elevator.ElevatorLevel.HATCHL2, Arm.ArmPosition.FLATBACKWARDS),
        HATCHL3FORWARDS(Elevator.ElevatorLevel.HATCHL3, Arm.ArmPosition.FLATFORWARDS),
        HATCHL3BACKWARDS(Elevator.ElevatorLevel.HATCHL3, Arm.ArmPosition.FLATBACKWARDS),
        CARGOL1FORWARDS(Elevator.ElevatorLevel.CARGO1, Arm.ArmPosition.FLATBACKWARDS),
        CARGOL1BACKWARDS(Elevator.ElevatorLevel.CARGO1, Arm.ArmPosition.FLATFORWARDS),
        CARGOSHIPFORWARDS(Elevator.ElevatorLevel.CARGOSHIP, Arm.ArmPosition.CARGOSHIPFORWARDS),
        CARGOSHIPBACKWARDS(Elevator.ElevatorLevel.CARGOSHIP, Arm.ArmPosition.CARGOSHIPBACKWARDS),
        CARGOL2FORWARDS(Elevator.ElevatorLevel.CARGOL2, Arm.ArmPosition.FLATBACKWARDS),
        CARGOL2BACKWARDS(Elevator.ElevatorLevel.CARGOL2, Arm.ArmPosition.FLATFORWARDS),
        CARGOL3FORWARDS(Elevator.ElevatorLevel.CARGOL3, Arm.ArmPosition.CARGOL3FRONT), // Move elevator first
        CARGOL3BACKWARDS(Elevator.ElevatorLevel.CARGOL3, Arm.ArmPosition.CARGOL3BACK), // move elevator first
        CARGOHANDOFF(Elevator.ElevatorLevel.CARGOHANDOFF, Arm.ArmPosition.CARGOHANDOFF), //ARM HIT, make sure cargo ground intake is deployed
        CARGOLOADINGSTATIONFWD(Elevator.ElevatorLevel.CARGOLOADINGSTATION, Arm.ArmPosition.FLATBACKWARDS),
        CARGOLOADINGSTATIONBWD(Elevator.ElevatorLevel.CARGOLOADINGSTATION, Arm.ArmPosition.FLATFORWARDS),
        STOWED(Elevator.ElevatorLevel.STOWED, Arm.ArmPosition.STOWED), //ARM HIT, Hatch intake is stowed
        VERTICALSTOWED(Elevator.ElevatorLevel.VERTICALSTOWED, Arm.ArmPosition.VERTICALSTOWED), //ARM HIT
        REVLIMITSWITCH(Elevator.ElevatorLevel.MINROTATE, Arm.ArmPosition.REVLIMITSWITCH),
        FWDLIMITSWITCH(Elevator.ElevatorLevel.MINROTATE, Arm.ArmPosition.FWDLIMITSWITCH),
        START(null, null),
        CARGOGROUNDINTAKE(null, null), 
        BOTTOMSTART(Elevator.ElevatorLevel.START, Arm.ArmPosition.FLATFORWARDS),
        CLIMB(Elevator.ElevatorLevel.MINROTATE, Arm.ArmPosition.CLIMB),
        BEFORECARGOHANDOFF(Elevator.ElevatorLevel.MINROTATE, Arm.ArmPosition.CARGOHANDOFF);

        public Elevator.ElevatorLevel eLevel;
        public Arm.ArmPosition armPos;
        ScoringPosition(Elevator.ElevatorLevel eLevel, Arm.ArmPosition armPos)
        {
            this.eLevel = eLevel;
            this.armPos = armPos;
        }
    }

    private enum Movement
    {
        ARRIVED,
        MOVEELEV,
        MOVEARM,
        SAFEZMOVE,
        SAFEZDMOVE,
        SAFEZUMOVE,
        FREEMOVE
    }

    public static void init()
    {
        //Variables to control init
        aimedRobotState = null;
        initializedRobot = false;
        initStep = 0;
        ranOnce = false;


        //Variables to control cargo intake
        arrivedAtMidPos=false;
        prevCargoIntakeExtended=false;

        // Variables to control climb
        shoppingCartDeployed = false;
        mopDeploy = false;
        extendedIntakeOnce = false;
        climbMode = false;
        climbStep = 0;
        startedBallIntakeTimer = false;

        //Init aimed state
        aimedRobotState = ScoringPosition.START;
    }

    public static void setControllers(Joysticks mainController, Joysticks coController)
    {
        if(!Robot.cargoDetection)
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
        else if(Robot.cargoDetection) //If the robot has a ball:
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

        if(coController.leftJoyStickPress)
            aimedRobotState = ScoringPosition.STOWED;

        if(coController.leftTrigger > .15)
            if(aimedRobotState == ScoringPosition.CARGOLOADINGSTATIONBWD || aimedRobotState == ScoringPosition.CARGOLOADINGSTATIONFWD)
                BallShooter.intakeCargo(1);
            else
                aimedRobotState = ScoringPosition.CARGOHANDOFF;
        else if(mainController.leftTrigger > .15)
        {
            //if(!elevatorManual)
                BallIntake.setOpenLoop(1);
        }
        else
        {
            ballIntakeTimer.reset();
            ballIntakeTimer.stop();
            arrivedAtMidPos = false;
            BallIntake.stopMotor();
        }

        if(coController.rightTrigger > .15)
            BallShooter.shootBall(coController.rightTrigger);
        else if(coController.leftTrigger < .15 && (Math.abs(Arm.encoderVelocity) > 500 || Math.abs(Elevator.encoderVelocity) > 500))
            BallShooter.intakeCargo(.45);
        else
            BallShooter.stopMotor();

        //Main controller controls
        if(mainController.rightTrigger > .3)
        {
            BallIntake.retract();
            prevCargoIntakeExtended = false;
            ballIntakeTimer.reset();
            ballIntakeTimer.stop();
        }

        if (mainController.buttonA)
        {
            aimedRobotState = null;
            Elevator.aimedState = ElevatorLevel.START;
        }
        else if (mainController.buttonY)
        {
            aimedRobotState = ScoringPosition.REVLIMITSWITCH;
        }

        if(mainController.leftMidButton && mainController.rightMidButton)
        {
            forceCargoOn = false;
            forceCargoOff = false;
        }
        else if(mainController.rightMidButton)
        {
            forceCargoOn = true;
            forceCargoOff = false;
        }
        else if(mainController.leftMidButton)
        {
            forceCargoOn = false;
            forceCargoOff = true;
        }

        if(mainController.dPadLeft)
            aimedRobotState = ScoringPosition.CARGOLOADINGSTATIONBWD;
        else if(mainController.dPadRight)
            aimedRobotState = ScoringPosition.CARGOLOADINGSTATIONFWD;
        // if(mainController.buttonX)
        // {
        //     aimedRobotState = ScoringPosition.CLIMB;
        // }
    }

    public static void run()
    {
        elevatorAboveMinRotate = Elevator.isAboveMinRotate(-550);          

        if(aimedRobotState != null)
        {
            boolean aimedStatesAreNotNull = aimedRobotState.eLevel!=null && aimedRobotState.armPos != null;

            if(aimedStatesAreNotNull)
            {
                elevatorReachedState = Elevator.reachedState(aimedRobotState.eLevel);
                armReachedState = Arm.reachedState(aimedRobotState.armPos);
            }
            else
            {
                elevatorReachedState = false;
                armReachedState = false;
            }
            elevatorAimedStateAboveMinRotate = aimedRobotState.eLevel != null ? Elevator.isStateAboveMinRotate(aimedRobotState.eLevel) : false;

            switch(aimedRobotState)
            {
                case START:
                    initializeRobotPosition();
                    break;
                case CARGOHANDOFF:
                    cargoHandoff();
                    break;
                case CLIMB:
                    climbing();
                    break;
                default:
                    if(aimedStatesAreNotNull)
                        goToAimedState();
                    break;

            }
        }
    }

    private static void goToAimedState()
    {
        goToAimedState(aimedRobotState);
    }

    private static void goToAimedState(ScoringPosition aimedState)
    {
        boolean aimedStateIsCargoHandoff = aimedState == ScoringPosition.CARGOHANDOFF;
        switch(movementCheck(aimedState))
        {
            case ARRIVED:
                Arm.aimedState = aimedState.armPos;
                Elevator.aimedState = aimedState.eLevel;
                if(aimedStateIsCargoHandoff)
                    runCargoHandoff();
                break;
            case MOVEELEV:
                Elevator.aimedState = aimedState.eLevel;
                break;
            case MOVEARM:
                Arm.aimedState = aimedState.armPos;
                break;
            case SAFEZMOVE:
                safetyRotateArm(aimedState.armPos);
                break;
            case SAFEZDMOVE:
                safetyRotateArmDown(aimedState.armPos);
                break;
            case SAFEZUMOVE:
                safetyRotateArmUp(aimedState.armPos, aimedState.eLevel);
                break;
            case FREEMOVE:
                Arm.aimedState = aimedState.armPos;
                Elevator.aimedState = aimedState.eLevel;
                break; 
        }
    }

    private static void runCargoHandoff()
    {
        prevCargoIntakeExtended = true;
        BallIntake.run();
        if(Robot.coController.leftTrigger < .15)
        {
            BallIntake.stopMotor();
            if(Robot.cargoDetection)
                aimedRobotState = ScoringPosition.CARGOSHIPFORWARDS;
            else
                aimedRobotState = ScoringPosition.BEFORECARGOHANDOFF;
        }
    }

    private static void extendCargoGroundIntake()
    {
        if(!arrivedAtMidPos)
        {
            goToAimedState(ScoringPosition.BEFORECARGOHANDOFF);
            if(Elevator.isAboveMinRotate(-500))
            {
                arrivedAtMidPos = true;
            }
            ballIntakeTimer.reset();
            startedBallIntakeTimer = false;
        }
        else
        {
            BallIntake.extend(); 
            BallIntake.stopMotor();
            if(!startedBallIntakeTimer)
            {
                ballIntakeTimer.start();
                startedBallIntakeTimer = true;
            }
        }
    }

    private static void cargoHandoff()
    {
        if(prevCargoIntakeExtended || ballIntakeTimer.get() > .5)
            goToAimedState(ScoringPosition.CARGOHANDOFF);
        else
            extendCargoGroundIntake();
    }

    //Method will only run when robot initializes until reaching hatch level 1 forwards
    // Cannot run again
    private static void initializeRobotPosition()
    {
        if(!ranOnce)
        {
            switch(initStep)
            {
                case 0:
                    safetyRotateArm(ArmPosition.REVLIMITSWITCH);
                    if(Arm.getRevLimitSwitch())
                    {
                        Arm.resetEncoder();
                        initStep = 1;
                    }
                    break;  
                case 1:
                    aimedRobotState = ScoringPosition.BOTTOMSTART;
                    break;
            }
        }

    }

    private static void safetyRotateArm(Arm.ArmPosition pos)
    {
        if( 
            Elevator.encoderVelocity > -750 && 
            elevatorAboveMinRotate && 
            Elevator.encoderValue <= 30000
          )
        {
            Arm.aimedState = pos;
        }
        else
        {
            Elevator.aimedState = Elevator.ElevatorLevel.MINROTATE;
            Arm.aimedState = ArmPosition.STOPPED;
        }
    }

    private static void safetyRotateArmDown(Arm.ArmPosition pos)
    {
        Elevator.aimedState = ElevatorLevel.MINROTATE;
        Arm.aimedState = pos;
    }

    private static void safetyRotateArmUp(Arm.ArmPosition armPos, Elevator.ElevatorLevel eLevel)
    {
        if(elevatorAboveMinRotate)
        {
            Arm.aimedState = armPos;
            Elevator.aimedState = eLevel;
        }
        else
        {
            Elevator.aimedState = Elevator.ElevatorLevel.MINROTATE;
            Arm.aimedState = ArmPosition.STOPPED;
        }
    }

    private static boolean inThreshold(int val, int actual, int threshold)
    {
        return (val > actual - threshold) && (val < actual + threshold);
    }

    private static void rotateArmClimb(Arm.ArmPosition pos)
    {
        if( Elevator.encoderValue >= Constants.elevatorHatchL2 - 500 && 
            Elevator.encoderValue <= 30000
          )
            Arm.aimedState = pos;
        else
        {
            Elevator.aimedState = Elevator.ElevatorLevel.HATCHL2;
            Arm.aimedState = ArmPosition.STOPPED;
        }
    }


    public static int possibeArmPosition(int elevEncoder)
    {
        if(Arm.encoderValue < Constants.armSRXVerticalStowed)
            return (int)( Math.pow((20018 - elevEncoder) / (.00022417), .5) + 13150);
        else
            return (int)( -Math.pow((20018 - elevEncoder) / (.00022417), .5) + 13150);
    }

    public static int possibleElevatorPosition(int armEncoder)
    {
        return (int)(-0.00022417 * (armEncoder - Constants.armSRXFlatForwards) * (armEncoder - Constants.armSRXFlatBackwards));
    }

    private static Movement movementCheck(SeriesStateMachine.ScoringPosition aimedState)
    {
        if(elevatorReachedState && armReachedState) 
        {
            return Movement.ARRIVED;
        }
        else if(!elevatorReachedState && armReachedState)//if arm is correct pos, elevator can ALWAYS move
        {
            return Movement.MOVEELEV;
        }
        else if(elevatorReachedState && !armReachedState)//check if arm can move without moving elevator
        {
            if(elevatorAboveMinRotate)
                return Movement.MOVEARM;
            else
                return Movement.SAFEZMOVE;
        }
        else //Both have to move
        {
            if(elevatorAboveMinRotate && elevatorAimedStateAboveMinRotate)
                return Movement.FREEMOVE;
            else if(elevatorAboveMinRotate && !elevatorAimedStateAboveMinRotate)
                return Movement.SAFEZDMOVE;
            else if(elevatorAimedStateAboveMinRotate)
                return Movement.SAFEZUMOVE;
            else
                return Movement.SAFEZMOVE;
        }
    }

    public static ScoringPosition getAimedRobotState()
    {
        return aimedRobotState;
    }

    public static void setAimedRobotState(ScoringPosition newAimedState)
    {
        aimedRobotState = newAimedState;
    }
    
    private static void climbing() 
    {
        if(!elevatorManual && inThreshold(Arm.encoderValue, Constants.armSRXClimb, 500) && inThreshold(Elevator.encoderValue, Constants.elevatorHatchL2, 1000))
        {
            // System.out.println("Arm reached Position");
            elevatorManual = false;
            switch (climbStep) 
            {
                case 0:
                    climbTimer.reset();
                    climbTimer.start();
                    // System.out.println("Climb timer started");
                    climbStep = 1;
                    break;
                case 1:
                    BallIntake.extend();
                    if (climbTimer.get() > 1.5)
                        climbStep = 2;
                    break;
                case 2:
                    // System.out.println("Deploying shopping cart!");
                    ShoppingCart.deployShoppingCart();
                    if (inThreshold(ShoppingCart.encoderValue, Constants.shoppingCartDeployed, 1000))
                        climbStep = 3;
                    break;
                case 3:
                    // System.out.println("retracting Ball Intake");
                    BallIntake.retract();
                    climbStep = 4;
                    break;
                case 4:
                    Mop.deployMop();
                    climbTimer.reset();
                    climbTimer.start();
                    climbStep = 5;
                    break;
                case 5:
                    if (climbTimer.get() > 2)
                        elevatorManual = true;
                    break;
            }
        }
        else if(elevatorManual)
        {
            Elevator.aimedState = null;
            Arm.aimedState = ArmPosition.CLIMB;
            climbMode = true;
            if (Robot.mainController.leftTrigger > .1) {
                Elevator.setOpenLoop(Robot.mainController.leftTrigger * .75);
            } else if (Robot.mainController.rightTrigger > .1) {
                Elevator.setOpenLoop(-Robot.mainController.rightTrigger);
            } else {
                Elevator.setOpenLoop(0);
            }
        }
        else
            rotateArmClimb(ArmPosition.CLIMB);
    }
}