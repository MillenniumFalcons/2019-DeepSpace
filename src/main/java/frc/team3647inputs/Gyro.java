package frc.team3647inputs;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.PigeonIMU;

public class Gyro
{
    PigeonIMU gyro = new PigeonIMU(0);  //Create new Pigeon Gyro Object

    double[] ypr = new double[3];

    private double yaw, pitch, roll, heading;     //Create public variables
    private ErrorCode errorCode;
    

    public void updateGyro()
    {
        errorCode = gyro.getYawPitchRoll(ypr);
        heading = gyro.getCompassHeading();
        yaw = ypr[0];
        pitch = ypr[1];
        roll = ypr[2];
    }

    public void resetAngle()
    {
        gyro.setCompassAngle(0);
    }

    public void testAngles()
    {
        System.out.println("Yaw: " + -yaw + "\nHeading: " + heading + "\nPitch: " + pitch + "\nRoll: " + roll);
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

    public double getHeading()
    {
        return this.heading;
    }
}