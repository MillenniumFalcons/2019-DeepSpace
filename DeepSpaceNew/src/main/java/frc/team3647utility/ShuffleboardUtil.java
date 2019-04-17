package frc.team3647utility;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShuffleboardUtil
{
    String[] structure = {"Rocket","CargoShip"};
    String[] direction = {"Left","Right"};
    String[] autoMode = {"Mixed","Regular"};

    String structureIn;
    String directionIn;
    String autoModeIn;

    public ShuffleboardUtil()
    {
        putStringArray("Structure", structure);
        putStringArray("Direction", direction);
        putStringArray("Auto Mode", autoMode);
    }

    public void putStringArray(String inputName, String[] input)
    {
        SmartDashboard.putStringArray(inputName, input);
    }

    public void getValues(String input)
    {
        structureIn = SmartDashboard.getString(input, "hello");
    }
    
}