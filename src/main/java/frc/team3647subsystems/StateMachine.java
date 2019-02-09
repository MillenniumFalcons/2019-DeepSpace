package frc.team3647subsystems;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.team3647pistons.IntakeBall;
import frc.team3647pistons.IntakeHatch;
import frc.team3647pistons.IntakeHatch.HatchPosition;
import frc.team3647subsystems.Arm.ArmPosition;
import frc.team3647subsystems.Elevator.ElevatorLevel;
import frc.team3647subsystems.team3647commands.*;

public class StateMachine
{
    //Combined Robot.elevator and Robot.arm System


    public void testCommand()
    {
        // addSequential(new setElevatorLevel(ElevatorLevel.MIDDLE));
    }

    public void rest()
    {
        /* The original state with everything at rest. 
        Robot.elevator: Bottom/ low / middle / max
        Robot.arm: 0 deg or any
        Robot.arm.Sensorfwd
        Robot.arm.sensorbwd
        BIntake: retracted / out
        BallArm: not-rolling / rolling fwd / rolling bwd
        HGrab: retracted / out
        IntakeHatch: Inside
        BBannerSensor: false

        HSensor: false
        */
        Robot.arm.setArmPosition(ArmPosition.STRAIGHT0);
        Robot.elevator.setElevatorLevel(ElevatorLevel.MIDDLE);
        IntakeBall.closeIntake();
        IntakeHatch.closeIntake();
    }

    // PS = player station
    // F = front
    // B = back

    public void hatchIntakePSF()
    {
        /* The intake of the hatch from player station from the front
        Robot.elevator: low
        Robot.arm: 0 deg
        However as for the HPiston and HGrab, there are a few steps:
        1.  HPiston: extracted      * Just the piston out in the hole *
            HGrab: retracted
        2.  HPiston: extracted      * The piston is in position and the grab
            Hgrab: exreacted        is extracted to hold on to the hatch *
        3.  HPiston: retracted      * The grab now holds on the ball so the 
            HGrab: extracted        grab will have to bring it back and store *
        
        At the end of this state, we should have the hatch stored and can be used to score
        */

        Robot.arm.setArmPosition(ArmPosition.STRAIGHT0);
        Robot.elevator.setElevatorLevel(ElevatorLevel.LOW);
        IntakeHatch.openIntake();
        IntakeHatch.closeIntake();
    }

    public void hatchIntakePSB()
    {
        /* The intake of the hatch from player station from the back
        Robot.elevator: low
        Robot.arm: 180 deg
        However as for the HPiston and HGrab, there are a few steps:
        1.  HPiston: extracted      * Just the piston out in the hole *
            HGrab: retracted
        2.  HPiston: extracted      * The piston is in position and the grab
            Hgrab: exreacted        is extracted to hold on to the hatch *
        3.  HPiston: retracted      * The grab now holds on the ball so the 
            HGrab: extracted        grab will have to bring it back and store *
        
        At the end of this state, we should have the hatch stored and can be used to score
        */

        Robot.arm.setArmPosition(ArmPosition.STRAIGHT180);
        Robot.elevator.setElevatorLevel(ElevatorLevel.LOW);
        IntakeHatch.openIntake();
        IntakeHatch.closeIntake();
    }

    public void hatchIntakeFloor()
    {
        /* The intake of the hatch from the floor (which can only be done from the front)
        Robot.arm: 0 deg
        The elevator and the hatch floor intake has several steps to follow:
        1.  Robot.elevator: mid                       * The elevator must be moved up for the HFGrab
            HFGrab: retracted completely        to come out *
            HGrab: retracted
        2.  Robot.elevator: mid                       * The HFGrab is now out to intake the hatch from
            HFGrab: extracted completely        the floor *
            HGrab: retracted
        * Manual control to get the hatch inside the intake *
        */

        Robot.arm.setArmPosition(ArmPosition.STRAIGHT0);
        IntakeHatch.setPosition(HatchPosition.OUTSIDE);
        Robot.elevator.setElevatorLevel(ElevatorLevel.LOW);
    }

    /**
     * this is called when a hatch is inside the floor intake and we want to intake it into the arm
     */
    public void hatchIntakeFloorLoad()
    {
        /*
        3.  Robot.elevator: low                       * The elevator in now in low position in order to score
            HFGrab: retracted in half way       the hatch so the HFGrab must be retracted half way and
            HGrab: extracted                    the HGrab in position to extract and keep hold of the hatch *
        4.  Robot.elevator mid                        * The hatch is now stored in the Robot.arm ready for scoring
            HFGrab: retracted completely        while the elevator must be moved up in order to retract
            HGrab: extracted                    the HFGrab completely
        
        At the end of this state, we should have the hatch stored and can be used to score
        */

        Robot.arm.setArmPosition(ArmPosition.STRAIGHT0);
        IntakeHatch.setPosition(HatchPosition.LOADING);
        Robot.elevator.setElevatorLevel(ElevatorLevel.LOW);
        //Add floor hatch to arm hatch intake
        Robot.elevator.setElevatorLevel(ElevatorLevel.MIDDLE); 

    }

    public void hatchLowScoreF()
    {
        /* The scoring of the hatch in low position from the front
        Robot.elevator: low
        Robot.arm: 0 deg
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

    public void hatchLowScoreB()
    {
        /* The scoring of the hatch in low position from the back
        Robot.elevator: low
        Robot.arm: 180 deg
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

    public void hatchMidScoreF()
    {
        /* The scoring of the hatch in mid position from the front
        Robot.elevator: mid
        Robot.arm: 0 deg
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

    public void hatchMidScoreB()
    {
        /* The scoring of the hatch in mid position from the back
        Robot.elevator: mid
        Robot.arm: 180 deg
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

    public void hatchHighScoreF()
    {
        /* The scoring of the hatch in high position from the front
        Robot.elevator: high
        Robot.arm: 0 deg
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

    public void hatchHighScoreB()
    {
        /* The scoring of the hatch in high position from the back
        Robot.elevator: high
        Robot.arm: 180 deg
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

    public void startPositionBall()
    {
        /*
        The ball has not been picked up yet, the ball intake and ball carrier are both empty.
        Robot.elevator: mid
        Robot.arm: 0 deg
        Intake: retracted
        Shoot: retracted
        BSensor: false
        */
    }
    public void intakeBall()
    {
        /*
        The ball is being picked up and the ball intake has a ball, and the ball carrier is empty.
        At this state, the robot is ready for intake but the process must be mostly done manually
        Robot.elevator: low
        Robot.arm: 0 deg
        Intake: extracted
        Shoot: retracted
        BSensor: false
        */
    }
    public void hasBall()
    {
        /*
        The ball is loaded in the carrier and is ready to score. 
        Robot.elevator: low
        Robot.arm: 0 deg
        Intake: retracted
        Shoot: retracted
        BSensor: true
        */
    }
    public void scoreLowBallF()
    {
        /*
        The ball is being scored at the lowest elevator level from the front
        Robot.elevator: low
        Robot.arm: 0 deg
        Intake: retracted
        Shoot: extracted
        BSensor: true
        */
    }
    public void scoreLowBallB()
    {
        /*
        The ball is being scored at the lowest elevator level from the back
        Robot.elevator: low
        Robot.arm: 180 deg
        Intake: retracted
        Shoot: extracted
        BSensor: true
        */
    }
    public void scoreMidBallF()
    {
        /*
        The ball is being scored at the middle elevator level from the front
        Robot.elevator: mid
        Robot.arm: 0 deg
        Intake: retracted
        Shoot: extracted
        BSensor: true
        */
    }
    public void scoreMidBallB()
    {
        /*
        The ball is being scored at the middle elevator level from the back
        Robot.elevator: mid
        Robot.arm: 180 deg
        Intake: retracted
        Shoot: extracted
        BSensor: true
        */
    }
    public void scoreHighBallF()
    {
        /*
        The ball is being scored at the highest elevator level from the front
        Robot.elevator: high
        Robot.arm: 0 deg
        Intake: retracted
        Shoot: extracted
        BSensor: true
        */
    }
    public void scoreHighBallB()
    {
        /*
        The ball is being scored at the highest elevator level from the back
        Robot.elevator: high
        Robot.arm: 180 deg
        Intake: retracted
        Shoot: extracted
        BSensor: true
        */
    }
    public void scoreCargoBallF()
    {
        /*
        The ball is being scored at the cargo elevator level from the front
        Robot.elevator: cargo
        Robot.arm: 180 deg
        Intake: retracted
        Shoot: extracted
        BSensor: true
        */
    }
    public void scoreCargoBallB()
    {
        /*
        The ball is being scored at the cargo elevator level from the back
        Robot.elevator: cargo
        Robot.arm: 180 deg
        Intake: retracted
        Shoot: extracted
        BSensor: true
        */
    }
}
