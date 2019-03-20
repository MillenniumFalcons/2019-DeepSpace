package frc.team3647utility;

import frc.team3647subsystems.*;
import frc.team3647subsystems.Arm.ArmPosition;
import frc.team3647subsystems.Elevator.ElevatorLevel;
import frc.robot.*;

import edu.wpi.first.wpilibj.Notifier;

public class RobotPos
{
    public ElevatorLevel eLevel;
    public ArmPosition armPos;

    public enum Movement
    {
        ARRIVED,
        MOVEELEV,
        MOVEARM,
        SAFEZMOVE,
        FREEMOVE
    }

    public RobotPos(ElevatorLevel eLevel, ArmPosition armPos)
    {
        this.eLevel = eLevel;
        this.armPos = armPos;
    }

    public Movement movementCheck(SeriesStateMachine.ScoringPosition scoringPos, RobotPos posData)
    {
        // System.out.println("CHECKING MOVEMENT: \nElev level: " + posData.eLevel + " \nArm Level: " + posData.armPos);
        if(Elevator.currentState == posData.eLevel && Arm.currentState == posData.armPos) 
        {
            return Movement.ARRIVED;
        }
        else if(Elevator.currentState != posData.eLevel && Arm.currentState == posData.armPos)//if arm is correct pos, elevator can ALWAYS move
        {
            return Movement.MOVEELEV;
        }
        else if(Elevator.currentState == posData.eLevel && Arm.currentState != posData.armPos)//check if arm can move without moving elevator
        {
            switch(scoringPos)
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
        //Both have to move
        else
        {
            switch(scoringPos)
            {
                //Change from SAFEZMOVE to only if statement
                case HATCHL1FORWARDS:
                    // if(Elevator.isAboveMinRotate(8000) && Elevator.elevatorEncoderVelocity < 3000)
                    //     return Movement.FREEMOVE;
                    // else
                        return Movement.SAFEZMOVE;
                case HATCHL1BACKWARDS:
                    // if(Elevator.isAboveMinRotate(8000) && Elevator.elevatorEncoderVelocity < 3000)
                    //     return Movement.FREEMOVE;
                    // else
                        return Movement.SAFEZMOVE;
                case STOWED:
                    // if(Elevator.isAboveMinRotate(8000) && Elevator.elevatorEncoderVelocity < 3000)
                    //     return Movement.FREEMOVE;
                    // else
                        return Movement.SAFEZMOVE;                    
                case CARGOHANDOFF:
                    // if(Elevator.isAboveMinRotate(8000) && Elevator.elevatorEncoderVelocity < 3000)
                    //     return Movement.FREEMOVE;
                    // else
                        return Movement.SAFEZMOVE;
                case VERTICALSTOWED:
                    // if(Elevator.isAboveMinRotate(8000) && Elevator.elevatorEncoderVelocity < 3000)
                    //     return Movement.FREEMOVE;
                    // else
                        return Movement.SAFEZMOVE;
                case CARGOL3BACKWARDS:
                    if(Elevator.isAboveMinRotate(0))
                        return Movement.FREEMOVE;
                    else
                        return Movement.SAFEZMOVE;
                default:
                    if(Elevator.elevatorEncoderVelocity > 1000 && Elevator.getStateEncoder(posData.eLevel) >= Constants.elevatorMinRotation && Elevator.isAboveMinRotate(0))
                        return Movement.FREEMOVE;
                    else
                    {
                        return Movement.SAFEZMOVE;
                    }
            }
        }
    }

    public Movement movementCheckExperimental(SeriesStateMachine.ScoringPosition scoringPos, RobotPos posData)
    {
        // Both mechanisms are at the correct location
        if(Arm.currentState == posData.armPos && Elevator.currentState == posData.eLevel) // No movement
        {
            return Movement.ARRIVED;
        }
        // Arm is at the correct position, elevator has to move
        else if(Arm.currentState == posData.armPos && Elevator.currentState != posData.eLevel) // Only elevator moves
        {
            return Movement.MOVEELEV;
        }
        // Elevator is at the correct position but arm has to move
        else if(Arm.currentState != posData.armPos && Elevator.currentState == posData.eLevel)
        {
            if(Elevator.elevatorEncoderValue >= Constants.elevatorMinRotation) // If elevator above minRotation freely move arm
            {
                return Movement.MOVEARM;
            }
            else if(Elevator.elevatorEncoderValue < Constants.elevatorMinRotation)
            { // Elevator below min rotation value

                // If elevator is moving up at a fast speed and is above a certain height, start moving the arm
                if(Elevator.elevatorEncoderValue > Constants.elevatorMinRotation / 1.4 && Elevator.elevatorEncoderVelocity > 2000)
                {
                    return Movement.MOVEARM;
                }
                // Otherwise, continue moving towards minRotation
                else
                {
                    return Movement.SAFEZMOVE;
                }
            }
            else
            {
                return Movement.SAFEZMOVE;
            }
        }
        // Elevator and arm have to move
        else
        {
            // Aimed state and current state are above minRotate for elevator
            if(Elevator.isAboveMinRotate(0) && Elevator.getStateEncoder(posData.eLevel) >= Constants.elevatorMinRotation)
            {
                return Movement.FREEMOVE;
            }
            // Elevator is above 29000 but is aimed at below minRotate
            else if(Elevator.isAboveMinRotate(10000) && Elevator.getStateEncoder(posData.eLevel) < Constants.elevatorMinRotation)
            {
                return Movement.FREEMOVE;
            }
            // Elevator is below minRotate but is going to be above min rotate
            else if(!Elevator.isAboveMinRotate(0) && Elevator.getStateEncoder(posData.eLevel) >= Constants.elevatorMinRotation)
            {
                if(Elevator.elevatorEncoderValue > Constants.elevatorMinRotation / 1.4 && Elevator.elevatorEncoderVelocity > 2000)
                {
                    return Movement.FREEMOVE;
                }
                // Otherwise, (continue to) move elevator towards aimedState, which is above minRotate
                else
                {
                    return Movement.MOVEELEV;
                }
            }
            // elevator is below min rotate and aimed state is below min rotate
            else if(!Elevator.isAboveMinRotate(0) && Elevator.getStateEncoder(posData.eLevel) < Constants.elevatorMinRotation)
            {
                return Movement.SAFEZMOVE;
            }
            else
            {
                return Movement.SAFEZMOVE;
            }
        }
    }
    public boolean threshold(int constant, int currentValue, int threshold)
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

    public void setRobotPos(ArmPosition armPos, ElevatorLevel eLevel)
    {
        this.armPos = armPos;
        this.eLevel = eLevel;
    }

    public RobotPos getRobotPos()
    {
        return new RobotPos(eLevel, armPos);
    }
}