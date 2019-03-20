//Class not created by Kunal Singla

package frc.robot;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.Compressor;

public class AirCompressor
{
	private static Compressor compressor = new Compressor(Constants.CompressorPCMPin);
	
	public static void runCompressor()
	{
		compressor.start();
	}

	public static void stopCompressor()
	{
		compressor.stop();
	}
}