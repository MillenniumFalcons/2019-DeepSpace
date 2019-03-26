package frc.robot;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.Compressor;

public class AirCompressor
{
	private static Compressor compressor = new Compressor(Constants.CompressorPCMPin);
	
	public static void run()
	{
		compressor.start();
	}

	public static void stop()
	{
		compressor.stop();
	}
}