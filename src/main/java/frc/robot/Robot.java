package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.team3647inputs.*;


public class Robot extends TimedRobot 
{

    Joysticks joysticks;
    
    @Override
    public void robotInit() 
    {
        joysticks = new Joysticks();

    }

    @Override
    public void robotPeriodic() 
    {
        
    }
    
    @Override
    public void autonomousInit() 
    {

    }
    
    @Override
    public void autonomousPeriodic() 
    {
      
    }

    @Override
    public void teleopInit() 
    {

    }
    
    @Override
    public void teleopPeriodic() 
    {

    }
    
    @Override
    public void testPeriodic() 
    {
        joysticks.updateJoysticks();
        joysticks.vibrate(1, .9, 1);
    }
}
