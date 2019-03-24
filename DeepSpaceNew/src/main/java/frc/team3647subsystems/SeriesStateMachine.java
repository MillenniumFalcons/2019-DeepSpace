package frc.team3647subsystems;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.*;
import frc.team3647inputs.*;
import frc.team3647subsystems.Arm.ArmPosition;
import frc.team3647subsystems.Elevator.ElevatorLevel;

public class SeriesStateMachine
{
    // Variables to control initialization
    private static boolean ranOnce = false, initializedRobot = false;
    private static int initStep = 1;

    // Variables to control climb
    private static boolean shoppingCartDeployed=false, mopDeploy=false, extendedIntakeOnce=false, elevatorManual = false, climbMode=false;
    private static int climbStep = 0;

    private static ScoringPosition aimedRobotState;

    public static boolean forceCargoOff = false;
    public static boolean forceCargoOn = false;

    //Variables to control ground cargo intake
    private static boolean arrivedAtMidPos=false, prevCargoIntakeExtended=false;

    private static Timer ballIntakeTimer = new Timer(), climbTimer = new Timer();


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
        CARGOL3FORWARDS(Elevator.ElevatorLevel.HATCHL3, Arm.ArmPosition.CARGOL3FRONT), // Move elevator first
        CARGOL3BACKWARDS(Elevator.ElevatorLevel.HATCHL3, Arm.ArmPosition.CARGOL3BACK), // move elevator first
        CARGOHANDOFF(Elevator.ElevatorLevel.CARGOHANDOFF, Arm.ArmPosition.CARGOHANDOFF), //ARM HIT, make sure cargo ground intake is deployed
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

    public static void seriesStateMachineInit()
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

        //Init aimed state
        aimedRobotState = ScoringPosition.START;
    }

    public static void setControllers(Joysticks mainController, Joysticks coController)
    {
        if(!BallShooter.cargoDetection() || HatchGrabber.hatchIn())
        {
            if(coController.buttonA)
                if(!HatchGrabber.hatchIn())
                    aimedRobotState = ScoringPosition.HATCHL1FORWARDS;
                else
                    aimedRobotState = ScoringPosition.HATCHL1VISIONFORWARDS;
            else if(coController.buttonX)
                aimedRobotState = ScoringPosition.HATCHL2FORWARDS;
            else if(coController.buttonB)
                aimedRobotState = ScoringPosition.HATCHL2FORWARDS;
            else if(coController.buttonY)
                aimedRobotState = ScoringPosition.HATCHL3FORWARDS;
            else if(coController.dPadDown)
                if(!HatchGrabber.hatchIn())
                    aimedRobotState = ScoringPosition.HATCHL1BACKWARDS;
                else
                    aimedRobotState = ScoringPosition.HATCHL1VISIONBACKWARDS;
            else if(coController.dPadLeft)
                aimedRobotState = ScoringPosition.HATCHL2BACKWARDS;
            else if(coController.dPadRight)
                aimedRobotState = ScoringPosition.HATCHL2BACKWARDS;
            else if(coController.dPadUp)
                aimedRobotState = ScoringPosition.HATCHL3BACKWARDS;
        }
        else if(BallShooter.cargoDetection())// If the robot has a ball:
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
        {
            aimedRobotState = ScoringPosition.STOWED;
        }

        if(coController.leftTrigger > .15)
        {  
            if (!arrivedAtMidPos) {
                ballIntakeTimer.reset();
                ballIntakeTimer.start();
                aimedRobotState = ScoringPosition.CARGOGROUNDINTAKE;
            } else if (prevCargoIntakeExtended || ballIntakeTimer.get() > .5) {
                aimedRobotState = ScoringPosition.CARGOHANDOFF;
            }
        } 
        else
        {
            arrivedAtMidPos = false;
            BallIntake.stopMotor();
        }

        if(coController.rightTrigger > .15)
        {
            BallShooter.shootBall(coController.rightTrigger);
        }
        else if(coController.leftTrigger < .15 && (Math.abs(Arm.armEncoderVelocity) > 500 || Math.abs(Elevator.elevatorEncoderVelocity) > 500))
        {
            BallShooter.intakeCargo(.45);
        }
        else
        {
            BallShooter.stopMotor();
        }

        //Main controller controls
        if(mainController.rightTrigger > .3)
        {
            BallIntake.retractIntake();
            prevCargoIntakeExtended = false;
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
            

    }

    public static void runSeriesStateMachine()
    {
        if(aimedRobotState != null)
        {
            switch(aimedRobotState)
            {
                case START:
                    initializeRobotPosition();
                    break;
                case CARGOGROUNDINTAKE:
                    extendCargoGroundIntake();
                    break;
                case CLIMB:
                    climbing();
                    break;
                default:
                    if(aimedRobotState.eLevel != null && aimedRobotState.armPos != null)
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
        switch(movementCheck(aimedState))
        {
            case ARRIVED:
                Arm.aimedState = aimedState.armPos;
                Elevator.aimedState = aimedState.eLevel;
                if(aimedState.equals(ScoringPosition.CARGOHANDOFF))
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
        BallIntake.runIntake();
        if(Robot.coController.leftTrigger < .15)
        {
            BallIntake.stopMotor();
            BallShooter.stopMotor();
            if(BallShooter.cargoDetection())
                aimedRobotState = ScoringPosition.CARGOSHIPFORWARDS;
            else
                aimedRobotState = ScoringPosition.BEFORECARGOHANDOFF;
        }
    }

    private static void climbing() 
    {
        if(!elevatorManual && inThreshold(Arm.armEncoderValue, Constants.armSRXClimb, 500) && inThreshold(Elevator.elevatorEncoderValue, Constants.elevatorHatchL2, 1000))
        {
            // System.out.println("Arm reached Position");
            elevatorManual = false;
            switch (climbStep) {
            case 0:
                climbTimer.reset();
                climbTimer.start();
                // System.out.println("Climb timer started");
                climbStep = 1;
                break;
            case 1:
                BallIntake.extendIntake();
                if (climbTimer.get() > 1.5)
                    climbStep = 2;
                break;
            case 2:
                // System.out.println("Deploying shopping cart!");
                ShoppingCart.deployShoppingCart();
                if (inThreshold(ShoppingCart.shoppingCartEncoderValue, Constants.shoppingCartDeployed, 1000))
                    climbStep = 3;
                break;
            case 3:
                // System.out.println("retracting Ball Intake");
                BallIntake.retractIntake();
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
            Arm.aimedState = ArmPosition.STOPPED;
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
                        Arm.resetArmEncoder();
                        initStep = 1;
                    }
                    break;  
                case 1:
                    aimedRobotState = ScoringPosition.BOTTOMSTART;
                    if(Arm.currentState == ArmPosition.FLATFORWARDS && Elevator.currentState == ElevatorLevel.BOTTOM)
                    {
                        ranOnce=true;
                        initializedRobot = true;
                    }
                    break;
            }
        }

    }

    private static void extendCargoGroundIntake()
    {
        if(!arrivedAtMidPos)
        {
            goToAimedState(ScoringPosition.BEFORECARGOHANDOFF);
            if(Elevator.isAboveMinRotate(0))
                arrivedAtMidPos = true;
        }
        if(arrivedAtMidPos)
            BallIntake.extendIntake(); 
    }

    private static void safetyRotateArm(Arm.ArmPosition pos)
    {
        if( 
            Elevator.elevatorEncoderVelocity > -750 && 
            Elevator.elevatorEncoderValue >= Constants.elevatorMinRotation - 500 && 
            Elevator.elevatorEncoderValue <= 30000
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
        if(Elevator.isAboveMinRotate(-500))
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
        if( Elevator.elevatorEncoderValue >= Constants.elevatorHatchL2 - 500 && 
            Elevator.elevatorEncoderValue <= 30000
          )
            Arm.aimedState = pos;
        else
        {
            Elevator.aimedState = Elevator.ElevatorLevel.HATCHL2;
            Arm.aimedState = ArmPosition.STOPPED;
        }
    }


    private static int possibeArmPosition(int elevEncoder)
    {
        if(Arm.armEncoderValue < Constants.armSRXVerticalStowed)
            return (int)( Math.pow((20018 - elevEncoder) / (.00022417), .5) + 13150);
        else
            return (int)( -Math.pow((20018 - elevEncoder) / (.00022417), .5) + 13150);
    }

    private static int possibleElevatorPosition(int armEncoder)
    {
        return (int)(-0.00022417 * (armEncoder - Constants.armSRXFlatForwards) * (armEncoder - Constants.armSRXFlatBackwards));
    }

    private static Movement movementCheck(SeriesStateMachine.ScoringPosition aimedState)
    {
        if(Elevator.currentState == aimedState.eLevel && Arm.currentState == aimedState.armPos) 
        {
            return Movement.ARRIVED;
        }
        else if(Elevator.currentState != aimedState.eLevel && Arm.currentState == aimedState.armPos)//if arm is correct pos, elevator can ALWAYS move
        {
            return Movement.MOVEELEV;
        }
        else if(Elevator.currentState == aimedState.eLevel && Arm.currentState != aimedState.armPos)//check if arm can move without moving elevator
        {
            if(Arm.isEncoderInThreshold() || Elevator.isAboveMinRotate(-500))
                return Movement.MOVEARM;
            else
                return Movement.SAFEZMOVE;
        }
        else //Both have to move
        {
            if(Elevator.isAboveMinRotate(-550) &&
                aimedState.eLevel.encoderVal >= Constants.elevatorMinRotation
               )
                return Movement.FREEMOVE;
            else if(Elevator.isAboveMinRotate(-550) &&
                    aimedState.eLevel.encoderVal < Constants.elevatorMinRotation
                    )
                return Movement.SAFEZDMOVE;
            else if(!Elevator.isAboveMinRotate(-500) && 
                    aimedState.eLevel.encoderVal >= Constants.elevatorMinRotation
                   )
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
}