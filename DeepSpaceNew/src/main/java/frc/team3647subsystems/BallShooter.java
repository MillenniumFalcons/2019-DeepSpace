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
		double mDemand = demand/2;
		if(mDemand < .3)
			mDemand = .5;
		setOpenLoop(limitCurrent(mDemand));
	}
	
	public static void intakeCargo(double power) //40 amps limit
	{

		setOpenLoop(-limitCurrent(.4));
	}

	private static double limitCurrent(double constant)
	{
		double current = Robot.pDistributionPanel.getCurrent(Constants.ballShooterPDPpin);
		return (1.3 / current + constant);
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