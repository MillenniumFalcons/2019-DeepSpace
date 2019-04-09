package frc.robot;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.Compressor;

public class AirCompressor
{
	private static Compressor compressor = new Compressor(Constants.CompressorPCMPin);
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