package frc.team3647inputs;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        updateLimelight();
    }

    public void setToDriver()
    {
        table.getEntry("camMode").setNumber(1); //Limelight Network Table code to set camera mode to driver vision
    }

    public void ledOff()
    {
        table.getEntry("ledMode").setNumber(1);
    }

    public void ledOn()
    {
        table.getEntry("ledMode").setNumber(3);
    }

    public void pip()
    {
        table.getEntry("stream").setNumber(2);
    }

    public void setToVison()
    {
        table.getEntry("ledMode").setNumber(0);
        table.getEntry("camMode").setNumber(0); //Limelight Network Table code to set camera mode to vision tracking
    }

    public void updateLimelight()
    {
        NetworkTableEntry tx = table.getEntry("tx"); //setting the N.T. entry to the tx value from limelight N.T.
		NetworkTableEntry ty = table.getEntry("ty"); //setting the N.T. entry to the ty value from limelight N.T.
        NetworkTableEntry ta = table.getEntry("ta"); //setting the N.T. entry to the ta value from limelight N.T.
        
		x = tx.getDouble(-3647);      //x is set to tx, and setting the default value to -3647 if not recieving values from limelight
		y = ty.getDouble(-3647);      //y is set to ty, and setting the default value to -3647 if not recieving values from limelight
		area = ta.getDouble(-3647);   //area is set to ta, and setting the default value to -3647 if not recieving values from limelight

		SmartDashboard.putNumber("LimelightX", x);          //adding the values to SmartDashboard
		SmartDashboard.putNumber("LimelightY", y);          //adding the values to SmartDashboard
		SmartDashboard.putNumber("LimelightArea", area);    //adding the values to SmartDashboard
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
}