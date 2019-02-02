package frc.team3647autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.team3647autonomous.team3647commands.*;

public class AutoSelector extends CommandGroup
{
    boolean test = true;

    public enum AutoMode
    {
        TEST_PATH, 
        OPEN_LOOP_DRIVE,
    }

    public enum StartPosition
    {
        LEFT,
        MIDDLE,
        RIGHT,
    }

    public AutoSelector(StartPosition startingPos, AutoMode auto)
    {
        switch(auto)
        {
            case TEST_PATH:
                System.out.println("Test Path Auto");
                addSequential(new TestAuto());
                break;
            case OPEN_LOOP_DRIVE:
                System.out.println("Open loop drive auto");
                addSequential(new OpenLoopDrive(0.2,0.5,5));
                break;
            default:
                System.out.println("no auto selected");
                break;
        }
    }
}