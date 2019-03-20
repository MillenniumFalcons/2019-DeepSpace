package frc.team3647subsystems;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.*;
import frc.team3647inputs.*;
import frc.team3647subsystems.Arm.ArmPosition;
import frc.team3647subsystems.Elevator.ElevatorLevel;

public class SeriesStateMachine
{
    // Variables to control initialization
    private static boolean arrivedAtRevLimitSwitchOnce=false, ranOnce = false;
    public static boolean initializedRobot = false;
    private static int initStep = 1;

    //control ball ground intake
    private static boolean intakeExtracted = false;
    private static boolean ballIntakeShouldBeRetracted = false;
    
    // Variables to control climb
    private static boolean shoppingCartDeployed=false, mopDeploy=false, extendedIntakeOnce=false;

	private static boolean climbMode=false;

    private static ScoringPosition aimedRobotState;

    private static Timer ballIntakeTimer, climbTimer;

    private static boolean specialMovement = false;
    private static int movementStep = 0;

    public enum ScoringPosition
    {
        HATCHL1FORWARDS(Elevator.ElevatorLevel.BOTTOM, Arm.ArmPosition.FLATFORWARDS), //ARM HIT
        HATCHL1BACKWARDS(Elevator.ElevatorLevel.BOTTOM, Arm.ArmPosition.FLATBACKWARDS), //ARM HIT
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
        BEFORECARGOHANDOFF(Elevator.ElevatorLevel.CARGOHANDOFF, Arm.ArmPosition.CARGOHANDOFF);

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
        FREEMOVE
    }

    public static void seriesStateMachineInit()
    {
        ballIntakeTimer = new Timer();
        climbTimer = new Timer();
        aimedRobotState = null;
        arrivedAtRevLimitSwitchOnce = false;
        initializedRobot = false;
        initStep = 0;
        ranOnce = false;
        intakeExtracted = false;

        // Variables to control climb
        shoppingCartDeployed = false;
        mopDeploy = false;
        extendedIntakeOnce = false;
        climbMode = false;
        climbStep = 0;

        ballIntakeShouldBeRetracted = false;

        specialMovement = false;
        movementStep = 0;

        aimedRobotState = ScoringPosition.START;
    }

    public static void initializeTeleop()
    {
        ballIntakeTimer = new Timer();
        climbTimer = new Timer();
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

        
        if(mainController.rightTrigger > .3)
        {
            BallIntake.retractIntake();
            ballIntakeShouldBeRetracted = true;
        }
        
        if(coController.leftTrigger > .15)
        {
            ballIntakeShouldBeRetracted = false;   
            //System.out.println("BallIntaketimer : " + ballIntakeTimer.get());
            if (!arrivedAtMidPos) {
                //System.out.println("Going to midPos");
                ballIntakeTimer.reset();
                ballIntakeTimer.start();
                aimedRobotState = ScoringPosition.CARGOGROUNDINTAKE;
            } else if (prevCargoIntakeExtended || ballIntakeTimer.get() > .5) {
                //System.out.println("Going to cargoHandoff");
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
        else if(BallShooter.cargoDetection() && coController.leftTrigger < .15 && Math.abs(Arm.armEncoderVelocity) > 500)
        {
            BallShooter.intakeCargo(.45);
        }
        else
        {
            BallShooter.stopMotor();
        }

        if (mainController.buttonA)
        {
            aimedRobotState = null;
            Elevator.aimedState = ElevatorLevel.START;
        }

        if (mainController.buttonY)
        {
            aimedRobotState = ScoringPosition.REVLIMITSWITCH;
        }
            
        if(coController.leftJoyStickPress)
        {
            aimedRobotState = ScoringPosition.STOWED;
        }

        // if(mainController.buttonX)
        // {
        //     Elevator.aimedState = null;
        //     Arm.aimedState = null;
        //     climbMode = true;
        //     aimedRobotState = ScoringPosition.CLIMB;
        // }
    }

    public static void runSeriesStateMachine()
    {
        // if(Arm.currentState != null && Elevator.currentState != null)
        //     robotState.setRobotPos(Arm.currentState, Elevator.currentState);
        // if (!climbMode)
        //     ShoppingCart.setPosition(0);
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
                case CARGOHANDOFF:
                    cargoHandoff();
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
        switch(movementCheck(aimedRobotState))
        {
            case ARRIVED:
                Arm.aimedState = aimedRobotState.armPos;
                Elevator.aimedState = aimedRobotState.eLevel;
                break;
            case MOVEELEV:
                Elevator.aimedState = aimedRobotState.eLevel;
                break;
            case MOVEARM:
                Arm.aimedState = aimedRobotState.armPos;
                break;
            case SAFEZMOVE:
                safetyRotateArm(aimedRobotState.armPos);
                break;
            case FREEMOVE:
                Arm.aimedState = aimedRobotState.armPos;
                Elevator.aimedState = aimedRobotState.eLevel;
                break; 
        }
    }

    private static void goToAimedState(ScoringPosition aimedState)
    {
        switch(movementCheck(aimedState))
        {
            case ARRIVED:
                Arm.aimedState = aimedState.armPos;
                Elevator.aimedState = aimedState.eLevel;
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
            case FREEMOVE:
                Arm.aimedState = aimedState.armPos;
                Elevator.aimedState = aimedState.eLevel;
                break; 
        }
    }

    private static void cargoHandoff()
    {
        switch(movementCheck(ScoringPosition.CARGOHANDOFF))
        {
            case ARRIVED:
                Arm.aimedState = ScoringPosition.CARGOHANDOFF.armPos;
                Elevator.aimedState = ScoringPosition.CARGOHANDOFF.eLevel;

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
                Elevator.aimedState = ScoringPosition.CARGOHANDOFF.eLevel;
                break;
            case MOVEARM:
                Arm.aimedState = ScoringPosition.CARGOHANDOFF.armPos;
                break;
            case SAFEZMOVE:
                safetyRotateArm(ScoringPosition.CARGOHANDOFF.armPos);
                break;
            case FREEMOVE:
                Arm.aimedState = ScoringPosition.CARGOHANDOFF.armPos;
                Elevator.aimedState = ScoringPosition.CARGOHANDOFF.eLevel;
                break;
            }
    }
    private static int climbStep = 0;
    private static boolean elevatorManual = false;
    private static void climbing() 
    {
        if(!elevatorManual && inThreshold(Arm.armEncoderValue, Constants.armSRXClimb, 500) && inThreshold(Elevator.elevatorEncoderValue, Constants.elevatorHatchL2, 1000))
        {
            System.out.println("Arm reached Position");
            elevatorManual = false;
            switch (climbStep) {
            case 0:
                climbTimer.reset();
                climbTimer.start();
                System.out.println("Climb timer started");
                climbStep = 1;
                break;
            case 1:
                BallIntake.extendIntake();
                if (climbTimer.get() > 1.5)
                    climbStep = 2;
                break;
            case 2:
                System.out.println("Deploying shopping cart!");
                ShoppingCart.deployShoppingCart();
                if (inThreshold(ShoppingCart.shoppingCartEncoderValue, Constants.shoppingCartDeployed, 1000))
                    climbStep = 3;
                break;
            case 3:
                System.out.println("retracting Ball Intake");
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
            Arm.aimedState = null;
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
        {
            rotateArmClimb(ArmPosition.CLIMB);
        }
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
                        arrivedAtRevLimitSwitchOnce = true;
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

    private static boolean arrivedAtMidPos=false, prevCargoIntakeExtended=false;
    private static void extendCargoGroundIntake()
    {
        if(!arrivedAtMidPos)
        {
            goToAimedState(ScoringPosition.BEFORECARGOHANDOFF);
            if(Elevator.isAboveMinRotate(0))
                arrivedAtMidPos = true;
        }
        if(arrivedAtMidPos)
        {
            BallIntake.extendIntake();
        }        
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
            Arm.aimedState = null;
        }
    }

    private static boolean inThreshold(int val,int actual, int threshold)
    {
        if((val > actual - threshold) && (val < actual + threshold))
            return true;
        else
            return false;
    }

    private static void rotateArmClimb(Arm.ArmPosition pos)
    {
        if(Elevator.elevatorEncoderValue >= Constants.elevatorHatchL2 - 500 && Elevator.elevatorEncoderValue <= 30000)
        {
            Arm.aimedState = pos;
        }
        else
        {
            Elevator.aimedState = Elevator.ElevatorLevel.HATCHL2;
            Arm.aimedState = null;
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
        return (int)(-0.00022417 * (armEncoder - 3700) * (armEncoder - 22600));
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
            switch(aimedState)
            {
                case HATCHL1FORWARDS: // fast movement!
                    if(threshold(Constants.armSRXFlatForwards, Arm.armEncoderValue, 800))
                        return Movement.MOVEARM;
                    else
                        return Movement.SAFEZMOVE;
                case HATCHL1BACKWARDS: // fast movement!
                    if(threshold(Constants.armSRXFlatBackwards, Arm.armEncoderValue, 500))
                        return Movement.MOVEARM;
                    else
                        return Movement.SAFEZMOVE;
                case STOWED:
                    if(threshold(Constants.armSRXStowed, Arm.armEncoderValue, 100))
                        return Movement.MOVEARM;
                    else
                        return Movement.SAFEZMOVE;                   
                case CARGOHANDOFF:
                    if(threshold(Constants.armSRXCargoHandoff, Arm.armEncoderValue, 100))
                        return Movement.MOVEARM;
                    else
                        return Movement.SAFEZMOVE;
                case VERTICALSTOWED:
                    if(threshold(Constants.armSRXFlatForwards, Arm.armEncoderValue, 100))
                        return Movement.MOVEARM;
                    else
                        return Movement.SAFEZMOVE;
                default:
                    if(Elevator.isAboveMinRotate(0))
                        return Movement.MOVEARM;
                    else
                        return Movement.SAFEZMOVE;
            }
        }
        else //Both have to move
        {
            switch(aimedState)
            {
                case HATCHL1FORWARDS: //Change from SAFEZMOVE to only if statement
                    return Movement.SAFEZMOVE;
                case HATCHL1BACKWARDS:
                    return Movement.SAFEZMOVE;
                case STOWED:
                    return Movement.SAFEZMOVE;                    
                case CARGOHANDOFF:
                    return Movement.SAFEZMOVE;
                case VERTICALSTOWED:
                    return Movement.SAFEZMOVE;
                case CARGOL3BACKWARDS:
                    if(Elevator.isAboveMinRotate(0))
                        return Movement.FREEMOVE;
                    else
                        return Movement.SAFEZMOVE;
                default:
                    if( Elevator.getStateEncoder(aimedState.eLevel) >= Constants.elevatorMinRotation && 
                        Elevator.isAboveMinRotate(0) )
                        return Movement.FREEMOVE;
                    else
                    {
                        return Movement.SAFEZMOVE;
                    }
            }
        }
    }

    private static boolean threshold(int constant, int currentValue, int threshold)
    {
        if((constant + threshold) > currentValue && (constant - threshold) < currentValue)
		{
			return true;
		}
		else
		{
			return false;
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


    public static void setControllers(Joysticks mainController, Guitar coController)
    {
        if(!BallShooter.cargoDetection())
        {

            if(coController.strumDown)
            {
                if(coController.fret1Up)
                    aimedRobotState = ScoringPosition.HATCHL1FORWARDS;
                else if(coController.fret2Up)
                    aimedRobotState = ScoringPosition.HATCHL2FORWARDS;
                else if(coController.fret3Up)
                    aimedRobotState = ScoringPosition.HATCHL3FORWARDS;
            }
            else if(coController.strumUp)
            {
                if(coController.fret1Up)
                    aimedRobotState = ScoringPosition.HATCHL1BACKWARDS;
                else if(coController.fret2Up)
                    aimedRobotState = ScoringPosition.HATCHL2BACKWARDS;
                else if(coController.fret3Up)
                    aimedRobotState = ScoringPosition.HATCHL3BACKWARDS;
            }
        }
        // If the robot has a ball:
        else if(BallShooter.cargoDetection())
        {
            if(coController.strumDown)
            {
                if(coController.fret1Up)
                    aimedRobotState = ScoringPosition.CARGOL1FORWARDS;
                else if(coController.fret2Up)
                    aimedRobotState = ScoringPosition.CARGOL2FORWARDS;
                else if(coController.fret2Down)
                    aimedRobotState = ScoringPosition.CARGOSHIPFORWARDS;
                else if(coController.fret3Up)
                    aimedRobotState = ScoringPosition.CARGOL3FORWARDS;
            }
            else if(coController.strumUp)
            {
                if(coController.fret1Up)
                    aimedRobotState = ScoringPosition.CARGOL1BACKWARDS;
                else if(coController.fret2Up)
                    aimedRobotState = ScoringPosition.CARGOL2BACKWARDS;
                else if(coController.fret2Down)
                    aimedRobotState = ScoringPosition.CARGOSHIPBACKWARDS;
                else if(coController.fret3Up)
                    aimedRobotState = ScoringPosition.CARGOL3BACKWARDS;
            }
        }


        if(mainController.rightTrigger > .3)
        {
            BallIntake.retractIntake();
        }
        
        if(coController.fret1Down && coController.strumUp)
        {
            //System.out.println("BallIntaketimer : " + ballIntakeTimer.get());
            if (!arrivedAtMidPos) {
                //System.out.println("Going to midPos");
                ballIntakeTimer.reset();
                ballIntakeTimer.start();
                aimedRobotState = ScoringPosition.CARGOGROUNDINTAKE;
            } else if (prevCargoIntakeExtended || ballIntakeTimer.get() > .5) {
                //System.out.println("Going to cargoHandoff");
                aimedRobotState = ScoringPosition.CARGOHANDOFF;
            }
        } 
        else
        {
            arrivedAtMidPos = false;
            BallIntake.stopMotor();
        }

        if(BallShooter.cargoDetection() && coController.fret1Down && coController.strumUp && Math.abs(Arm.armEncoderVelocity) > 500)
        {
            BallShooter.intakeCargo(.45);
        }
        else
        {
            BallShooter.stopMotor();
        }
        
        if(coController.fret3Down && coController.strumUp)
        {
            BallShooter.shootBall(.7);
        }
        else if(!(coController.fret3Down && coController.strumUp) && !BallShooter.cargoDetection())
        {
            BallShooter.stopMotor();
        }

        if (mainController.buttonA)
        {
            aimedRobotState = null;
            Elevator.aimedState = ElevatorLevel.START;
        }

        if (mainController.buttonY)
        {
            aimedRobotState = ScoringPosition.REVLIMITSWITCH;
        }
            
        if(coController.stow)
        {
            aimedRobotState = ScoringPosition.STOWED;
        }
    }
    
}