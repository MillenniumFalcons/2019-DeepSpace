package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3647inputs.Joysticks;
import frc.team3647inputs.Limelight;
import frc.team3647subsystems.VisionController;

///TO BE MOVED SOMETIME, ELSEWHERE
public class TestFunctions 
{

	public static void deletePIDFMM() 
	{
		SmartDashboard.delete("kPright"); // 0.3473
		SmartDashboard.delete("kIright"); // 0
		SmartDashboard.delete("kDright"); // 0.7389
		SmartDashboard.delete("kFright");

		SmartDashboard.delete("kPleft");
		SmartDashboard.delete("kIleft");
		SmartDashboard.delete("kDleft");
		SmartDashboard.delete("kFleft");

		// SmartDashboard.delete("kP", 0.45);
		// SmartDashboard.delete("kI", 0.035);
		// SmartDashboard.delete("kD", 0.9);
		// SmartDashboard.delete("kF", 0.0);

		SmartDashboard.delete("MM Acceleration");
		SmartDashboard.delete("MM Velocity");

	}

	public static void updatePIDFMM() 
	{
		// Robot.PIDFright[0] = SmartDashboard.getNumber("kPright", 0);
		// Robot.PIDFright[1] = SmartDashboard.getNumber("kIright", 0);
		// Robot.PIDFright[2] = SmartDashboard.getNumber("kDright", 0);
		// Robot.PIDFright[3] = SmartDashboard.getNumber("kFright", 0.26);

		// Robot.PIDFleft[0] = SmartDashboard.getNumber("kPleft", 0);
		// Robot.PIDFleft[1] = SmartDashboard.getNumber("kIleft", 0);
		// Robot.PIDFleft[2] = SmartDashboard.getNumber("kDleft", 0);
		// Robot.PIDFleft[3] = SmartDashboard.getNumber("kFleft", 0.26);

		Robot.PIDF[0] = SmartDashboard.getNumber("kP", 0.45);
		Robot.PIDF[1] = SmartDashboard.getNumber("kI", 0.035);
		Robot.PIDF[2] = SmartDashboard.getNumber("kD", 0.9);
		Robot.PIDF[3] = SmartDashboard.getNumber("kF", 0.0);

		// Robot.mVel = (int)SmartDashboard.getNumber("MM Velocity", 1000);
		// Robot.mAccel = (int)SmartDashboard.getNumber("MM Acceleration", 1000);
	}

	public static void shuffleboardInit() 
	{

		// SmartDashboard.putNumber("kPright", 0); //0.3473
		// SmartDashboard.putNumber("kIright", 0); //0
		// SmartDashboard.putNumber("kDright", 0); //0.7389
		// SmartDashboard.putNumber("kFright", .26);

		// SmartDashboard.putNumber("kPleft", 0);
		// SmartDashboard.putNumber("kIleft", 0);
		// SmartDashboard.putNumber("kDleft", 0);
		// SmartDashboard.putNumber("kFleft", 0.26);

		SmartDashboard.putNumber("kP", 0.45);
		SmartDashboard.putNumber("kI", 0.035);
		SmartDashboard.putNumber("kD", 0.9);
		SmartDashboard.putNumber("kF", 0.0);

		// SmartDashboard.putNumber("MM Acceleration", 1000);
		// SmartDashboard.putNumber("MM Velocity", 1000);
	}

	/*********************************************/

	public static void antiTipping() 
	{
		if (tipping()) 
		{
			dont();
		}
	}

	public static void dont() 
	{
		// put code here
	}

	public static boolean tipping() 
	{
		double tipValue = 50;
		double pitch = Math.abs(Robot.gyro.getPitch());
		double roll = Math.abs(Robot.gyro.getRoll());
		if (pitch > tipValue || roll > tipValue) 
		{
			return true;
		} 
		else
			return false;
	}

	

}