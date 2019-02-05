package frc.team3647autonomous.team3647commands;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.team3647autonomous.RamseteFollower;

public class FollowPath extends Command 
{
  RamseteFollower ramseteFollower;
  Notifier pathThread;
  Timer stopWatch = new Timer();

  public FollowPath(String path, boolean backwards) 
  {
    if (backwards == true) 
    {
      requires(Robot.drivetrain);
      ramseteFollower = new RamseteFollower(path, backwards);
      System.out.println("Created Ramsete follower");
      pathThread = new Notifier(() -> {
      ramseteFollower.runPathBackwards();
      });
    } 
    else
    {
      requires(Robot.drivetrain);
      ramseteFollower = new RamseteFollower(path, backwards);
      System.out.println("Created Ramsete follower");
      pathThread = new Notifier(() -> {
      ramseteFollower.runPath();
      });
    }

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() 
  {
    Robot.encoders.resetEncoders();
    stopWatch.start();
    System.out.println("Initialized path");
    pathThread.startPeriodic(0.02);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() 
  {
    System.out.println("Path currently running");
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() 
  {
    return ramseteFollower.isFinished();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() 
  {
    pathThread.stop();
    System.out.println("Path Successfully completed!");
    stopWatch.stop();
    System.out.println("Time to complete path: " + stopWatch.get());
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() 
  {
    System.out.println("Path interrupted!");
  }
}
