package frc.robot;

import frc.robot.Constants;
import frc.team3647utility.CustomCompressor;

public class AirCompressor
{
	private static CustomCompressor compressor = new CustomCompressor(Constants.CompressorPCMPin);
	public static boolean running = false;
	
	public static void run()
	{
		running = true;
		compressor.start();
	}

	public static void stop()
	{
		running = false;
		compressor.stop();
	}
}