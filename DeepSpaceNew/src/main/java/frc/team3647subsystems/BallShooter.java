package frc.team3647subsystems;

import frc.robot.*;
import frc.team3647subsystems.Canifier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class BallShooter 
{
  private static VictorSPX intakeMotor = new VictorSPX(Constants.ballShooterPin);
	
	public static void ballShooterinitialization()
	{
		intakeMotor.setInverted(false);
	}
	
	private static void setOpenLoop(double demand)
	{
		intakeMotor.set(ControlMode.PercentOutput, demand);
  }
    
  public static void stopMotor()
	{
		setOpenLoop(0);
	}

	public static void shootBall(double demand)
	{
		setOpenLoop(demand);
	}
	
	public static void intakeCargo(double power)
	{
		setOpenLoop(-1);
	}

	public static boolean cargoDetection()
	{
    return Canifier.cargoBeamBreak();
	}

	public static void printBeamBreak()
	{
		System.out.println("Cargo Beam Break: " + cargoDetection());
	}
}