package frc.team3647subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.GeneralPin;
import com.ctre.phoenix.CANifier.LEDChannel;
import frc.robot.*;

public class Canifier
{
    static CANifier can = new CANifier(Constants.canifierPin);

    public static void setLights(int r, int g, int b)
    {
        can.setLEDOutput(r, LEDChannel.LEDChannelA);
        can.setLEDOutput(g, LEDChannel.LEDChannelB);
        can.setLEDOutput(b, LEDChannel.LEDChannelC);
    }

    public static boolean cargoBeamBreak()
    {
        return !can.getGeneralInput(GeneralPin.LIMR);
    }

    public static boolean hatchGrabberSensor()
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
}
