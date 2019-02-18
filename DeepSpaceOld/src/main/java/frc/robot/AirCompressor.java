//Class not created by Kunal Singla

package frc.robot;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.Compressor;

public class AirCompressor
{
	static Compressor c = new Compressor(Constants.CompressorPCMPin);
	
	public static void runCompressor()
	{
	    c.setClosedLoopControl(true);
	}
}