package frc.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import frc.team3647inputs.*;


public class Robot extends IterativeRobot 
{

    Joysticks joystick;
    @Override
    public void robotInit() 
    {
        joystick = new Joysticks();

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
        joystick.updateJoysticks();
        joystick.vibrate();     

    }
}
