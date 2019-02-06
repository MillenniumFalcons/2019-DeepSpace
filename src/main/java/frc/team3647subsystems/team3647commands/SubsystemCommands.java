package frc.team3647subsystems.team3647commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.team3647subsystems.Elevator;
import frc.team3647subsystems.Elevator.ElevatorLevel;

public class SubsystemCommands extends Command
{
    @Override
    protected boolean isFinished() 
    {
        return false;
    }

    public void setElevatorLevel(ElevatorLevel level)
    {
        requires(Robot.elevator);
        Robot.elevator.setElevatorLevel(level);
    }
}