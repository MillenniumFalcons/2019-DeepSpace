package frc.team3647pistons;

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