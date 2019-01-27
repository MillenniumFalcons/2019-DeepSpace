package frc.team3647utility;

import frc.robot.Constants;

public interface Units
{
    public static double ticksToMeters(double ticks)
    {
        return (ticks / Constants.kEncoderTicks) * (inchesToMeters(Constants.kWheelDiameter * 3.14));
    }

    public static double inchesToMeters(double in)
    {
        return in * 0.0254;
    }

    public static double metersToInches(double m)
    {
        return m/0.0254;
    }

    public static double feetToMeters(double ft)
    {
        return inchesToMeters(ft*12);
    }

    public static double metersToFeet(double m)
    {
        return metersToInches(m) / 12;
    }

    public static double degreesToRadian(double d)
    {
        return Math.toRadians(d);
    }

    public static double radiansToDegress(double r)
    {
        return Math.toDegrees(r);
    }
}
