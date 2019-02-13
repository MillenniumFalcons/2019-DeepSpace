// package frc.team3647subsystems;

// import edu.wpi.first.wpilibj.command.CommandGroup;
// import frc.robot.Robot;
// import frc.team3647subsystems.*;
// import frc.team3647subsystems.Arm.ArmPosition;
// import frc.team3647subsystems.Elevator.ElevatorLevel;
// import java.util.Arrays;


// public class StateMachine
// {

//     //Combined Robot.Elevator and Robot.arm System
//     public Enum[] currentState = new Enum[]{Elevator.getCurrentState(), arm.getCurrentState(), intakeBall.getCurrentState(), intakeHatch.getCurrentState()};
    
//     public Enum[] aimedState = new Enum[]{Elevator.getAimedState(), arm.getAimedState(), intakeBall.getAimedState(), intakeHatch.getAimedState()};
    


//     private boolean homePositionChecker()
//     {
//         if(currentState[0] == ElevatorLevel.Home && currentState[1] == ArmPosition.StraightForwards && currentState[2] == IntakeBallState.RETRACTED && currentState[3] == HatchPosition.INSIDE)
//         {
//             return true;
//         }
//         else
//             return false;
//     }

//     public void homePosition()
//     {
//         if(ElevatorInLowZone( Elevator.getCurrentState() ) == true)
//         {
//             Elevator.setElevatorPosition(ElevatorLevel.Home);
//         }
//         else if(Elevator.getCurrentState() != ElevatorLevel.Home)
//         {
//             arm.runArm(ArmPosition.StraightForwards);
//         }
//         else if(currentState[0] !=  ElevatorLevel.Home && currentState[1] == ArmPosition.StraightForwards)
//         {
//             Elevator.setElevatorPosition(ElevatorLevel.Home);
//         }
//         else if(currentState[0] ==  ElevatorLevel.Home && currentState[1] != ArmPosition.StraightForwards)
//         {
//             arm.runArm(ArmPosition.StraightForwards);
//         }
//         else if(currentState[0] ==  ElevatorLevel.Home && currentState[1] == ArmPosition.StraightForwards)
//         {
//             intakeBall.closeIntake();
//             intakeHatch.closeIntake();
//             intakeHatch.setPosition(HatchPosition.INSIDE);;
//         }
//     }

//     public void stow()
//     {
//         setStateIntakesClosed(ElevatorLevel.RobotStowed, ArmPosition.RobotStowed);
//     }

//     public void hatchLvl1F()
//     {
//         setStateIntakesClosed(ElevatorLevel.HatchLevel1, ArmPosition.StraightForwards);
//     }

//     public void hatchLvl1B()
//     {
//         setStateIntakesClosed(ElevatorLevel.HatchLevel1, ArmPosition.StraightBackwards);
//     }

//     public void hatchLvl2F()
//     {
//         setStateIntakesClosed(ElevatorLevel.HatchLevel2, ArmPosition.StraightForwards);
//     }

//     public void hatchLvl2B()
//     {
//         setStateIntakesClosed(ElevatorLevel.HatchLevel2, ArmPosition.StraightBackwards);
//     }

//     public void hatchLvl3F()
//     {
//         setStateIntakesClosed(ElevatorLevel.HatchLevel3, ArmPosition.StraightForwards);
//     }

//     public void hatchLvl3B()
//     {
//         setStateIntakesClosed(ElevatorLevel.HatchLevel3, ArmPosition.StraightBackwards);
//     }

//     public void cargoLvl3F()
//     {
//         setStateIntakesClosed(ElevatorLevel.CargoLevel3, ArmPosition.CargoLevel3Front);
//     }

//     public void cargoLvl3B()
//     {
//         setStateIntakesClosed(ElevatorLevel.CargoLevel3, ArmPosition.StraightBackwards);
//     }

//     public void cargoLvl1F()
//     {
//         setStateIntakesClosed(ElevatorLevel.CargoLevel1, ArmPosition.StraightForwards);
//     }

//     public void cargoLvl1B()
//     {
//         setStateIntakesClosed(ElevatorLevel.CargoLevel1, ArmPosition.StraightBackwards);
//     }

//     public void cargoHandoff()
//     {
//         setStateBallIntake(ElevatorLevel.CargoHandoff, ArmPosition.CargoHandoff);
//     }

//     /****************HATCH STUFF THAT NEEDS TO BE FIXED***************/

//     public void startHatchGroundIntake()
//     {
//         arm.runArm(ArmPosition.HatchIntakeMovement);                        //Change to appropriate Arm pos
//         Elevator.setElevatorPosition(ElevatorLevel.HatchIntakeMovement);    //Change to appropriate Elevator pos
//         intakeBall.closeIntake();
//         intakeHatch.closeIntake();
//         //hatch floor intake slightly out
//     }
    
//     public void hatchGroundScore()
//     {
//         arm.runArm(ArmPosition.HatchHandoff);                       //Fix for appropriate Arm position
//         Elevator.setElevatorPosition(ElevatorLevel.HatchHandoff);   //Fix for appropriate Elevator position
//         intakeBall.closeIntake();
//         intakeHatch.setPosition(HatchPosition.LOADING);
//         //hatch floor intake OUT HALF WAY
//     }

//     public void hatchGroundIntake()
//     {
//         arm.runArm(ArmPosition.HatchIntakeMovement);                //Fix for appropriate Arm position
//         Elevator.setElevatorPosition(ElevatorLevel.HatchIntakeMovement);    //Fix for appropriate Elevator position
//         intakeBall.closeIntake();
//         intakeHatch.closeIntake();
//         //hatch floor intake OUT ALL THE WAY
//     }

//     public void hatchHandoff()
//     {
//         arm.runArm(ArmPosition.HatchHandoff);
//         Elevator.setElevatorPosition(ElevatorLevel.HatchHandoff);
//         intakeBall.closeIntake();
//         intakeHatch.setPosition(HatchPosition.LOADING);
//         //hatch floor intake HALF WAY
//     }

    
//     private boolean ElevatorInLowZone(ElevatorLevel level)
//     {
//         //To detect if arm can spin freely or not
//         //HatchLevel1,
// 		// CargoLevel1,
// 		// HatchHandoff,
// 		// CargoHandoff,
//         // HatchIntakeMovement,
//         // RobotStowed //Assuming it can't move freely in this position
//         switch(level)
//         {
//             //arm cannot move freely if:
//             case HatchLevel1:
//                 return true;
//             case CargoLevel1:
//                 return true;
//             case HatchHandoff:
//                 return true;
//             case CargoHandoff:
//                 return true;
//             case HatchIntakeMovement:
//                 return true;
//             case RobotStowed:
//                 return true;
//             default: //else arm can move freely
//                 return false;
//         }
//     }

//     /** Ball and Hatch Intake are inside */
//     private void setStateIntakesClosed(ElevatorLevel level, ArmPosition position)
//     {
//         if(homePositionChecker() == false)
//         {
//             homePosition();
//         }
//         else if(currentState[0] != aimedState[0] && currentState[1] != aimedState[1])
//         {
//             arm.runArm(position);
//         }
//         else if(currentState[0] != aimedState[0] && currentState[1] == aimedState[1])
//         {
//             Elevator.setElevatorPosition(level);
//         }
//         else if(currentState[0] == aimedState[0] && currentState[1] == aimedState[1])
//         {
//             intakeBall.closeIntake();
//             intakeHatch.setPosition(HatchPosition.INSIDE);
//             intakeHatch.closeIntake();
//         }
//     }

//     /** Ball Intake Outside and Hatch Intake inside */
//     private void setStateBallIntake(ElevatorLevel level, ArmPosition position)
//     {
//         if(homePositionChecker() == false)
//         {
//             homePosition();
//         }
//         else if(currentState[0] != aimedState[0] && currentState[1] != aimedState[1])
//         {
//             arm.runArm(position);
//         }
//         else if(currentState[0] != aimedState[0] && currentState[1] == aimedState[1])
//         {
//             Elevator.setElevatorPosition(level);
//         }
//         else if(currentState[0] == aimedState[0] && currentState[1] == aimedState[1])
//         {
//             intakeBall.openIntake();
//             intakeHatch.setPosition(HatchPosition.INSIDE);
//             intakeHatch.closeIntake();
//         }
//     }


// }
