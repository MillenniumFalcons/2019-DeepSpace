package frc.team3647subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.GeneralPin;
import com.ctre.phoenix.CANifier.LEDChannel;

public class Canifier
{
    static CANifier can = new CANifier(1);

    public static void setLights(int r, int g, int b)
    {
        can.setLEDOutput(r, LEDChannel.LEDChannelA);
        can.setLEDOutput(g, LEDChannel.LEDChannelB);
        can.setLEDOutput(b, LEDChannel.LEDChannelC);
    }

    public static boolean cargoBeamBrake()
    {
        if(can.getGeneralInput(GeneralPin.LIMF))
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    public static boolean hatchGrabberSensor()
    {
        if(can.getGeneralInput(GeneralPin.LIMR))
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}
