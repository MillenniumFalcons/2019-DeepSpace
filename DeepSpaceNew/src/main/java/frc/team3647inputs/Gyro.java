package frc.team3647inputs;

import com.ctre.phoenix.sensors.PigeonIMU;
import frc.robot.Constants;

public class Gyro
{
    /**
     * Pigeon IMU Object (a gyro sensor) to receive and reset Yaw, Pitch, Roll, and Heading
     */
    public PigeonIMU gyro = new PigeonIMU(Constants.gyroPin);  //Create new Pigeon Gyro Object

    /**
     * array of type double that holds Yaw (0), Pitch(1), and Roll(2) values (in degrees).
     */
    double[] ypr = new double[3];

    /**
     * gyro value
     */
    private double yaw, pitch, roll, heading, compassHeading;
    
    /**
     * This method updates the gyro values for Yaw, Pitch, Roll, and Heading.
     */
    public void update()
    {
        gyro.getYawPitchRoll(ypr);
        heading = gyro.getCompassHeading();
        compassHeading = gyro.getAbsoluteCompassHeading();
        yaw = ypr[0];
        pitch = ypr[1];
        roll = ypr[2];
    }

    /**
     * Resets angles to 0 degrees.
     */
    public void resetAngle()
    {
        gyro.setYaw(0);
    }

    public void setHeading(double angleDeg)
    {
        gyro.setFusedHeading(angleDeg);
    }

    public void setYaw(double angleDeg)
    {
        gyro.setYaw(angleDeg);
    }

    public void resetHeading()
    {
        this.setHeading(0);
    }

    public void setCompass(double angleDeg)
    {
        gyro.setCompassAngle(angleDeg);
    }

    public void resetCompass()
    {
        this.setCompass(0);
    }

    public void reset()
    {
        this.resetAngle();
        this.resetCompass();
        this.resetHeading();
    }

    /**
     * Prints Yaw, Pitch, Roll, and Heading
     */
    public void printAngles()
    {
        System.out.println("Yaw: " + -yaw + "\nHeading: " + heading + "\nPitch: " + pitch + "\nRoll: " + roll);
    }

    /**
     * @return Yaw Value
     */
    public double getYaw()
    {
        return this.yaw;
    }
    
    /**
     * @return Pitch Value
     */
    public double getPitch()
    {
        return this.pitch;
    }

    /**
     * @return Roll Value
     */
    public double getRoll()
    {
        return this.roll;
    }

    /**
     * @return Heading Value (should be similar to Yaw)
     */
    public double getHeading()
    {
        return this.heading;
    }

    public double getCompassHeading()
    {
        return this.compassHeading;
    }
}