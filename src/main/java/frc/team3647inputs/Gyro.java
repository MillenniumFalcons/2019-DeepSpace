package frc.team3647inputs;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.*;

public class Gyro
{
    PigeonIMU gyro = new PigeonIMU(0);

    public double yaw, pitch, roll;

    public void setAngle()
    {
        yaw = gyro.getCompassHeading();
    }

    public void resetAngle()
    {
        gyro.setCompassAngle(0);
    }

    public void testAngle()
    {
        System.out.println("Yaw: " + -yaw + " Pitch: " + pitch + " Roll: " + roll);
    }

}