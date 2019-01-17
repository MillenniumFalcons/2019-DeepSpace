package frc.team3647inputs;

import com.ctre.phoenix.sensors.PigeonIMU;

public class Gyro
{
    PigeonIMU gyro = new PigeonIMU(0);  //Create new Pigeon Gyro Object

    private double yaw, pitch, roll;     //Create public variables

    public void updateYaw()
    {
        yaw = gyro.getCompassHeading();
    }

    public void resetAngle()
    {
        gyro.setCompassAngle(0);
    }

    public void testAngles()
    {
        System.out.println("Yaw: " + -yaw + " Pitch: " + pitch + " Roll: " + roll);
    }

    public double getYaw()
    {
        return this.yaw;
    }
    
    public double getPitch()
    {
        return this.pitch;
    }

    public double getRoll()
    {
        return this.roll;
    }

}