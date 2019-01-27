package frc.team3647autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.team3647autonomous.team3647commands.*;

public class TestAuto extends CommandGroup 
{
  //test ye autos here
  public TestAuto() 
  {
    addSequential(new FollowPath("StraightFiveMAndCurveLeft", true));
  }
}
