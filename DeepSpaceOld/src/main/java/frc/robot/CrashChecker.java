package frc.robot;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.UUID;

public class CrashChecker 
{
	private static final UUID RUN_INSTANCE_UUID = UUID.randomUUID();
	
	public static void logRobotInit() 
	{
        logMarker("Robot Initialization", null);
    }
	
	public static void logAutoInit() 
	{
        logMarker("Autonomous Initialization", null);
    }
	
	public static void logTeleopPeriodic() 
	{
        logMarker("Tele-Operated Period", null);
    }
	
	public static void logThrowableCrash(Throwable throwable) 
	{
        logMarker("Exception", throwable);
    }
	
	//Pretty much a copy paste from Team 254
	//Props to them for creating this genius method
	private static void logMarker(String mark, Throwable nullableException) 
	{
		try (PrintWriter writer = new PrintWriter(new FileWriter("/home/lvuser/crash_tracking.txt", true))) 
		{
			writer.print(RUN_INSTANCE_UUID.toString());
			writer.print(", ");
			writer.print(mark);
			writer.print(", ");
			
			if (nullableException != null) 
			{
				writer.print(", ");
				nullableException.printStackTrace(writer);
			}
			
	        writer.println(); 
		} 
		catch (IOException e)
		{
			e.printStackTrace();
		}
	}
}