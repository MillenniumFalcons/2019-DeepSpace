package frc.team3647autonomous.team3647commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.*;

public class OpenLoopDrive extends Command 
{
  double lOutput, rOutput, timeMS;
  public static Timer stopWatch = new Timer();

  public OpenLoopDrive(double l, double r, double ms) 
  {
    requires(Robot.drivetrain);
    timeMS = ms;
    rOutput = r;
    lOutput = l;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize()
  {
    stopWatch.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() 
  {
    if(stopWatch.get() < timeMS)
    {
      Robot.drivetrain.setVelocity(lOutput, rOutput);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() 
  {
    return stopWatch.get() == timeMS;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() 
  {
    stopWatch.stop();
    Robot.drivetrain.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() 
  {

  }
}
