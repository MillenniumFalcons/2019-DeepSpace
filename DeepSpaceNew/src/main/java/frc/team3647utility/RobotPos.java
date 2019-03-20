// package frc.team3647utility;

// import frc.team3647subsystems.*;
// import frc.team3647subsystems.Arm.ArmPosition;
// import frc.team3647subsystems.Elevator.ElevatorLevel;
// import frc.robot.*;


// public class RobotPos
// {
//     public ElevatorLevel eLevel;
//     public ArmPosition armPos;

//     public enum Movement
//     {
//         ARRIVED,
//         MOVEELEV,
//         MOVEARM,
//         SAFEZMOVE,
//         FREEMOVE
//     }

//     public RobotPos(ElevatorLevel eLevel, ArmPosition armPos)
//     {
//         this.eLevel = eLevel;
//         this.armPos = armPos;
//     }

//     public Movement movementCheck(SeriesStateMachine.ScoringPosition scoringPos, RobotPos posData)
//     {
//         // System.out.println("CHECKING MOVEMENT: \nElev level: " + posData.eLevel + " \nArm Level: " + posData.armPos);
//         if(Elevator.currentState == posData.eLevel && Arm.currentState == posData.armPos) 
//         {
//             return Movement.ARRIVED;
//         }
//         else if(Elevator.currentState != posData.eLevel && Arm.currentState == posData.armPos)//if arm is correct pos, elevator can ALWAYS move
//         {
//             return Movement.MOVEELEV;
//         }
//         else if(Elevator.currentState == posData.eLevel && Arm.currentState != posData.armPos)//check if arm can move without moving elevator
//         {
//             switch(scoringPos)
//             {
//                 case HATCHL1FORWARDS: // fast movement!
//                     if(threshold(Constants.armSRXFlatForwards, Arm.armEncoderValue, 800))
//                         return Movement.MOVEARM;
//                     else
//                         return Movement.SAFEZMOVE;
//                 case HATCHL1BACKWARDS: // fast movement!
//                     if(threshold(Constants.armSRXFlatBackwards, Arm.armEncoderValue, 500))
//                         return Movement.MOVEARM;
//                     else
//                         return Movement.SAFEZMOVE;
//                 case STOWED:
//                     if(threshold(Constants.armSRXStowed, Arm.armEncoderValue, 100))
//                         return Movement.MOVEARM;
//                     else
//                         return Movement.SAFEZMOVE;                   
//                 case CARGOHANDOFF:
//                     if(threshold(Constants.armSRXCargoHandoff, Arm.armEncoderValue, 100))
//                         return Movement.MOVEARM;
//                     else
//                         return Movement.SAFEZMOVE;
//                 case VERTICALSTOWED:
//                     if(threshold(Constants.armSRXFlatForwards, Arm.armEncoderValue, 100))
//                         return Movement.MOVEARM;
//                     else
//                         return Movement.SAFEZMOVE;
//                 default:
//                     if(Elevator.isAboveMinRotate(0))
//                         return Movement.MOVEARM;
//                     else
//                         return Movement.SAFEZMOVE;
//             }
//         }
//         //Both have to move
//         else
//         {
//             switch(scoringPos)
//             {
//                 case HATCHL1FORWARDS:
//                     return Movement.SAFEZMOVE;
//                 case HATCHL1BACKWARDS:
//                     return Movement.SAFEZMOVE;
//                 case STOWED:
//                     return Movement.SAFEZMOVE;                    
//                 case CARGOHANDOFF:
//                     return Movement.SAFEZMOVE;
//                 case VERTICALSTOWED:
//                     return Movement.SAFEZMOVE;
//                 case CARGOL3BACKWARDS:
//                     if(Elevator.isAboveMinRotate(0))
//                         return Movement.FREEMOVE;
//                     else
//                         return Movement.SAFEZMOVE;
//                 default:
//                     if(Elevator.elevatorEncoderVelocity > 1000 && Elevator.getStateEncoder(posData.eLevel) >= Constants.elevatorMinRotation && Elevator.isAboveMinRotate(0))
//                         return Movement.FREEMOVE;
//                     else
//                     {
//                         return Movement.SAFEZMOVE;
//                     }
//             }
//         }
//     }

//     public Movement movementCheck(SeriesStateMachine.ScoringPosition aimedState)
//     {
//         // System.out.println("CHECKING MOVEMENT: \nElev level: " + posData.eLevel + " \nArm Level: " + posData.armPos);
//         if(Elevator.currentState == aimedState.eLevel && Arm.currentState == aimedState.armPos) 
//         {
//             return Movement.ARRIVED;
//         }
//         else if(Elevator.currentState != aimedState.eLevel && Arm.currentState == aimedState.armPos)//if arm is correct pos, elevator can ALWAYS move
//         {
//             return Movement.MOVEELEV;
//         }
//         else if(Elevator.currentState == aimedState.eLevel && Arm.currentState != aimedState.armPos)//check if arm can move without moving elevator
//         {
//             switch(aimedState)
//             {
//                 case HATCHL1FORWARDS: // fast movement!
//                     if(threshold(Constants.armSRXFlatForwards, Arm.armEncoderValue, 800))
//                         return Movement.MOVEARM;
//                     else
//                         return Movement.SAFEZMOVE;
//                 case HATCHL1BACKWARDS: // fast movement!
//                     if(threshold(Constants.armSRXFlatBackwards, Arm.armEncoderValue, 500))
//                         return Movement.MOVEARM;
//                     else
//                         return Movement.SAFEZMOVE;
//                 case STOWED:
//                     if(threshold(Constants.armSRXStowed, Arm.armEncoderValue, 100))
//                         return Movement.MOVEARM;
//                     else
//                         return Movement.SAFEZMOVE;                   
//                 case CARGOHANDOFF:
//                     if(threshold(Constants.armSRXCargoHandoff, Arm.armEncoderValue, 100))
//                         return Movement.MOVEARM;
//                     else
//                         return Movement.SAFEZMOVE;
//                 case VERTICALSTOWED:
//                     if(threshold(Constants.armSRXFlatForwards, Arm.armEncoderValue, 100))
//                         return Movement.MOVEARM;
//                     else
//                         return Movement.SAFEZMOVE;
//                 default:
//                     if(Elevator.isAboveMinRotate(0))
//                         return Movement.MOVEARM;
//                     else
//                         return Movement.SAFEZMOVE;
//             }
//         }
//         //Both have to move
//         else
//         {
//             switch(aimedState)
//             {
//                 //Change from SAFEZMOVE to only if statement
//                 case HATCHL1FORWARDS:
//                     return Movement.SAFEZMOVE;
//                 case HATCHL1BACKWARDS:
//                     return Movement.SAFEZMOVE;
//                 case STOWED:
//                     return Movement.SAFEZMOVE;                    
//                 case CARGOHANDOFF:
//                     return Movement.SAFEZMOVE;
//                 case VERTICALSTOWED:
//                     return Movement.SAFEZMOVE;
//                 case CARGOL3BACKWARDS:
//                     if(Elevator.isAboveMinRotate(0))
//                         return Movement.FREEMOVE;
//                     else
//                         return Movement.SAFEZMOVE;
//                 default:
//                     if(Elevator.elevatorEncoderVelocity > 1000 && Elevator.getStateEncoder(aimedState.eLevel) >= Constants.elevatorMinRotation && Elevator.isAboveMinRotate(0))
//                         return Movement.FREEMOVE;
//                     else
//                     {
//                         return Movement.SAFEZMOVE;
//                     }
//             }
//         }
//     }
//     public boolean threshold(int constant, int currentValue, int threshold)
//     {
//         if((constant + threshold) > currentValue && (constant - threshold) < currentValue)
// 		{
// 			return true;
// 		}
// 		else
// 		{
// 			return false;
// 		}
//     }

//     public void setRobotPos(ArmPosition armPos, ElevatorLevel eLevel)
//     {
//         this.armPos = armPos;
//         this.eLevel = eLevel;
//     }

//     public RobotPos getRobotPos()
//     {
//         return new RobotPos(eLevel, armPos);
//     }
// }