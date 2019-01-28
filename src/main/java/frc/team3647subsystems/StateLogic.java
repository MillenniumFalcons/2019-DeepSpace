package frc.team3647subsystems;

import frc.team3647pistons.IntakeBall;
import frc.team3647pistons.IntakeHatch;
import frc.team3647pistons.IntakeHatch.HatchPosition;
import frc.team3647subsystems.Arm.ArmPosition;
import frc.team3647subsystems.Elevator.ElevatorLevel;;

public class StateLogic
{
    //Combined Elevator and Arm System

    public static void rest()
    {
        /* The original state with everything at rest. 
        Elevator: mid
        Arm: 0 deg
        BIntake: retracted
        BShoot: retracted
        BSensor: false
        HPiston: retracted
        HGrab: retracted
        HFloorGrab: retracted
        HSensor: false
        */
        Arm.setArmPosition(ArmPosition.STRAIGHT0);
        Elevator.setElevatorLevel(ElevatorLevel.MIDDLE);
        IntakeBall.closeIntake();
        IntakeHatch.closeIntake();
    }

    // PS = player station
    // F = front
    // B = back

    public static void hatchIntakePSF()
    {
        /* The intake of the hatch from player station from the front
        Elevator: low
        Arm: 0 deg
        However as for the HPiston and HGrab, there are a few steps:
        1.  HPiston: extracted      * Just the piston out in the hole *
            HGrab: retracted
        2.  HPiston: extracted      * The piston is in position and the grab
            Hgrab: exreacted        is extracted to hold on to the hatch *
        3.  HPiston: retracted      * The grab now holds on the ball so the 
            HGrab: extracted        grab will have to bring it back and store *
        
        At the end of this state, we should have the hatch stored and can be used to score
        */

        Arm.setArmPosition(ArmPosition.STRAIGHT0);
        Elevator.setElevatorLevel(ElevatorLevel.LOW);
        IntakeHatch.openIntake();
        IntakeHatch.closeIntake();
    }

    public static void hatchIntakePSB()
    {
        /* The intake of the hatch from player station from the back
        Elevator: low
        Arm: 180 deg
        However as for the HPiston and HGrab, there are a few steps:
        1.  HPiston: extracted      * Just the piston out in the hole *
            HGrab: retracted
        2.  HPiston: extracted      * The piston is in position and the grab
            Hgrab: exreacted        is extracted to hold on to the hatch *
        3.  HPiston: retracted      * The grab now holds on the ball so the 
            HGrab: extracted        grab will have to bring it back and store *
        
        At the end of this state, we should have the hatch stored and can be used to score
        */

        Arm.setArmPosition(ArmPosition.STRAIGHT180);
        Elevator.setElevatorLevel(ElevatorLevel.LOW);
        IntakeHatch.openIntake();
        IntakeHatch.closeIntake();
    }

    public static void hatchIntakeFloor()
    {
        /* The intake of the hatch from the floor (which can only be done from the front)
        Arm: 0 deg
        The elevator and the hatch floor intake has several steps to follow:
        1.  Elevator: mid                       * The elevator must be moved up for the HFGrab
            HFGrab: retracted completely        to come out *
            HGrab: retracted
        2.  Elevator: mid                       * The HFGrab is now out to intake the hatch from
            HFGrab: extracted completely        the floor *
            HGrab: retracted
        * Manual control to get the hatch inside the intake *
        */

        Arm.setArmPosition(ArmPosition.STRAIGHT0);
        IntakeHatch.setPosition(HatchPosition.OUTSIDE);
        Elevator.setElevatorLevel(ElevatorLevel.LOW);
    }

    /**
     * this is called when a hatch is inside the floor intake and we want to intake it into the arm
     */
    public static void hatchIntakeFloorLoad()
    {
        /*
        3.  Elevator: low                       * The elevator in now in low position in order to score
            HFGrab: retracted in half way       the hatch so the HFGrab must be retracted half way and
            HGrab: extracted                    the HGrab in position to extract and keep hold of the hatch *
        4.  Elevator mid                        * The hatch is now stored in the Arm ready for scoring
            HFGrab: retracted completely        while the elevator must be moved up in order to retract
            HGrab: extracted                    the HFGrab completely
        
        At the end of this state, we should have the hatch stored and can be used to score
        */

        Arm.setArmPosition(ArmPosition.STRAIGHT0);
        IntakeHatch.setPosition(HatchPosition.LOADING);
        Elevator.setElevatorLevel(ElevatorLevel.LOW);
        //Add floor hatch to arm hatch intake
        Elevator.setElevatorLevel(ElevatorLevel.MIDDLE); 

    }

    public static void hatchLowScoreF()
    {
        /* The scoring of the hatch in low position from the front
        Elevator: low
        Arm: 0 deg
        The HPiston and HGrab will go through a process in order to score the hatch:
        1.  HPiston: retracted                  * The piston is retracted while the grab is still
            HGrab: extracted                    holding onto the hatch *
        2.  HPiston: extracted                  * The piston is now extracted to place in the desired
            HGrab: retracted                    area while the grab then retracte to let go of the hatch *
        3.  HPiston :retracted                  * Now that the hatch is placed on the desired location,
            HGrba: retracted                    the grab and piston are now both retracted to let go *

        At the end of this state, the hatch will be scored and the robot go back to rest position
        and ready to intake the ball or hatch
        */
    }

    public static void hatchLowScoreB()
    {
        /* The scoring of the hatch in low position from the back
        Elevator: low
        Arm: 180 deg
        The HPiston and HGrab will go through a process in order to score the hatch:
        1.  HPiston: retracted                  * The piston is retracted while the grab is still
            HGrab: extracted                    holding onto the hatch *
        2.  HPiston: extracted                  * The piston is now extracted to place in the desired
            HGrab: retracted                    area while the grab then retracte to let go of the hatch *
        3.  HPiston :retracted                  * Now that the hatch is placed on the desired location,
            HGrba: retracted                    the grab and piston are now both retracted to let go *

        At the end of this state, the hatch will be scored and the robot go back to rest position
        and ready to intake the ball or hatch
        */
    }

    public static void hatchMidScoreF()
    {
        /* The scoring of the hatch in mid position from the front
        Elevator: mid
        Arm: 0 deg
        The HPiston and HGrab will go through a process in order to score the hatch:
        1.  HPiston: retracted                  * The piston is retracted while the grab is still
            HGrab: extracted                    holding onto the hatch *
        2.  HPiston: extracted                  * The piston is now extracted to place in the desired
            HGrab: retracted                    area while the grab then retracte to let go of the hatch *
        3.  HPiston :retracted                  * Now that the hatch is placed on the desired location,
            HGrba: retracted                    the grab and piston are now both retracted to let go *

        At the end of this state, the hatch will be scored and the robot go back to rest position
        and ready to intake the ball or hatch
        */
    }

    public static void hatchMidScoreB()
    {
        /* The scoring of the hatch in mid position from the back
        Elevator: mid
        Arm: 180 deg
        The HPiston and HGrab will go through a process in order to score the hatch:
        1.  HPiston: retracted                  * The piston is retracted while the grab is still
            HGrab: extracted                    holding onto the hatch *
        2.  HPiston: extracted                  * The piston is now extracted to place in the desired
            HGrab: retracted                    area while the grab then retracte to let go of the hatch *
        3.  HPiston :retracted                  * Now that the hatch is placed on the desired location,
            HGrba: retracted                    the grab and piston are now both retracted to let go *

        At the end of this state, the hatch will be scored and the robot go back to rest position
        and ready to intake the ball or hatch
        */
    }

    public static void hatchHighScoreF()
    {
        /* The scoring of the hatch in high position from the front
        Elevator: high
        Arm: 0 deg
        The HPiston and HGrab will go through a process in order to score the hatch:
        1.  HPiston: retracted                  * The piston is retracted while the grab is still
            HGrab: extracted                    holding onto the hatch *
        2.  HPiston: extracted                  * The piston is now extracted to place in the desired
            HGrab: retracted                    area while the grab then retracte to let go of the hatch *
        3.  HPiston :retracted                  * Now that the hatch is placed on the desired location,
            HGrba: retracted                    the grab and piston are now both retracted to let go *

        At the end of this state, the hatch will be scored and the robot go back to rest position
        and ready to intake the ball or hatch
        */
    }

    public static void hatchHighScoreB()
    {
        /* The scoring of the hatch in high position from the back
        Elevator: high
        Arm: 180 deg
        The HPiston and HGrab will go through a process in order to score the hatch:
        1.  HPiston: retracted                  * The piston is retracted while the grab is still
            HGrab: extracted                    holding onto the hatch *
        2.  HPiston: extracted                  * The piston is now extracted to place in the desired
            HGrab: retracted                    area while the grab then retracte to let go of the hatch *
        3.  HPiston :retracted                  * Now that the hatch is placed on the desired location,
            HGrba: retracted                    the grab and piston are now both retracted to let go *

        At the end of this state, the hatch will be scored and the robot go back to rest position
        and ready to intake the ball or hatch
        */
    }

    public static void startPositionBall()
    {
        /*
        The ball has not been picked up yet, the ball intake and ball carrier are both empty.
        Elevator: mid
        Arm: 0 deg
        Intake: retracted
        Shoot: retracted
        BSensor: false
        */
    }
    public static void intakeBall()
    {
        /*
        The ball is being picked up and the ball intake has a ball, and the ball carrier is empty.
        At this state, the robot is ready for intake but the process must be mostly done manually
        Elevator: low
        Arm: 0 deg
        Intake: extracted
        Shoot: retracted
        BSensor: false
        */
    }
    public static void hasBall()
    {
        /*
        The ball is loaded in the carrier and is ready to score. 
        Elevator: low
        Arm: 0 deg
        Intake: retracted
        Shoot: retracted
        BSensor: true
        */
    }
    public static void scoreLowBallF()
    {
        /*
        The ball is being scored at the lowest elevator level from the front
        Elevator: low
        Arm: 0 deg
        Intake: retracted
        Shoot: extracted
        BSensor: true
        */
    }
    public static void scoreLowBallB()
    {
        /*
        The ball is being scored at the lowest elevator level from the back
        Elevator: low
        Arm: 180 deg
        Intake: retracted
        Shoot: extracted
        BSensor: true
        */
    }
    public static void scoreMidBallF()
    {
        /*
        The ball is being scored at the middle elevator level from the front
        Elevator: mid
        Arm: 0 deg
        Intake: retracted
        Shoot: extracted
        BSensor: true
        */
    }
    public static void scoreMidBallB()
    {
        /*
        The ball is being scored at the middle elevator level from the back
        Elevator: mid
        Arm: 180 deg
        Intake: retracted
        Shoot: extracted
        BSensor: true
        */
    }
    public static void scoreHighBallF()
    {
        /*
        The ball is being scored at the highest elevator level from the front
        Elevator: high
        Arm: 0 deg
        Intake: retracted
        Shoot: extracted
        BSensor: true
        */
    }
    public static void scoreHighBallB()
    {
        /*
        The ball is being scored at the highest elevator level from the back
        Elevator: high
        Arm: 180 deg
        Intake: retracted
        Shoot: extracted
        BSensor: true
        */
    }
    public static void scoreCargoBallF()
    {
        /*
        The ball is being scored at the cargo elevator level from the front
        Elevator: cargo
        Arm: 180 deg
        Intake: retracted
        Shoot: extracted
        BSensor: true
        */
    }
    public static void scoreCargoBallB()
    {
        /*
        The ball is being scored at the cargo elevator level from the back
        Elevator: cargo
        Arm: 180 deg
        Intake: retracted
        Shoot: extracted
        BSensor: true
        */
    }
}
