package frc.team3647inputs;

import frc.team3647subsystems.VisionController.VisionMode;
import edu.wpi.first.networktables.*;

public class Limelight
{
    private double x, y, area;
    //x is the tx degree value from Limelight Network Table
    //y is the ty degree value from Limelight Network Table
    //area is the ta value from Limelight Network Table. The ratio of the object area to the entire picture area, as a percent.

    public NetworkTable table;  // NetworkTable is the class used to grab values from the Limelight Network Table


    public Limelight(String orientation)  //used to initalize the main, important things
    {
        table = NetworkTableInstance.getDefault().getTable("limelight-" + orientation);    //initializing the network table to grab values from limelight
        update();
    }

    // public void setToDriver()
    // {
    //     table.getEntry("camMode").setNumber(1); //Limelight Network Table code to set camera mode to driver vision
    // }

    // public void ledPipeline()
    // {
    //     table.getEntry("ledMode").setNumber(0);
    // }
    // public void ledOff()
    // {
    //     table.getEntry("ledMode").setNumber(1);
    // }

    // public void ledOn()
    // {
    //     table.getEntry("ledMode").setNumber(3);
    // }
    public void set(VisionMode mode)
    {
        setPipeline(mode.pipeline);
    }
    
    private void setPipeline(int pipeline)
    {
        set("pipeline", pipeline);
    }

    public void update()
    {
        // NetworkTableEntry tx = table.getEntry("tx"); //setting the N.T. entry to the tx value from limelight N.T.
		// NetworkTableEntry ty = table.getEntry("ty"); //setting the N.T. entry to the ty value from limelight N.T.
        // NetworkTableEntry ta = table.getEntry("ta"); //setting the N.T. entry to the ta value from limelight N.T.
        
		x = get("tx");      //x is set to tx, and setting the default value to -3647 if not recieving values from limelight
		y = get("ty");      //y is set to ty, and setting the default value to -3647 if not recieving values from limelight
		area = get("ta");   //area is set to ta, and setting the default value to -3647 if not recieving values from limelight

		// SmartDashboard.putNumber("LimelightX", x);          //adding the values to SmartDashboard
		// SmartDashboard.putNumber("LimelightY", y);          //adding the values to SmartDashboard
		// SmartDashboard.putNumber("LimelightArea", area);    //adding the values to SmartDashboard
    }

    public double getX()            //get x, because x is private
    {
        return this.x;
    }

    public double getY()            //get y, because y is private
    {
        return this.y;
    }

    public double getArea()         //get area, because area is private
    {
        return this.area;
    }

    public void blink()
    {
        set("ledMode", 2);
    }

    public void pipeLineLED()
    {
        set("ledMode", 0);
    }

    public boolean getValidTarget()         //get area, because area is private
    {
        return get("tv") == 1;
    }

    private double get(String input)
    {
        return table.getEntry(input).getDouble(-3647);
    }

    public void set(String input, int input2)
    {
        table.getEntry(input).setNumber(input2);
    }

}