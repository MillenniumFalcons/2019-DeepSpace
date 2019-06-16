package frc.robot;

import frc.robot.Constants;
import frc.team3647utility.Compressor;

public class AirCompressor {
	private static Compressor compressor = new Compressor(Constants.CompressorPCMPin);
	public static boolean running = false;

	public static void run() {
		compressor.start();
		running = true;
	}

	public static void stop() {
		compressor.stop();
		running = false;
	}
}