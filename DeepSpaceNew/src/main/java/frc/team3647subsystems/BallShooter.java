package frc.team3647subsystems;

import frc.robot.*;
import frc.team3647autonomous.AutonomousSequences;
import frc.team3647subsystems.Canifier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Timer;

public class BallShooter 
{
	
	private static VictorSPX intakeMotor = new VictorSPX(Constants.ballShooterPin);

	private static boolean flashed = false, startedTimer = false;
	private static Timer ballBlinkTimer = new Timer();
	
	
	public static void ballShooterinitialization()
	{
		intakeMotor.setInverted(false);
		
	}

	public static void runBlink()
	{
		if(Robot.cargoDetection)
		{
			if(!startedTimer)
			{
				ballBlinkTimer.reset();
				ballBlinkTimer.start();
				startedTimer = true;
			}
			else if(!flashed && startedTimer && ballBlinkTimer.get() > .2)
			{
				AutonomousSequences.limelightClimber.limelight.blink();
				AutonomousSequences.limelightFourBar.limelight.blink();
				if(ballBlinkTimer.get() > 1.2)
				{
					flashed = true;
				}
			}
			if(flashed)
			{
				AutonomousSequences.limelightClimber.limelight.pipeLineLED();
				AutonomousSequences.limelightFourBar.limelight.pipeLineLED();
			}
		}
		else
		{
			ballBlinkTimer.reset();
			ballBlinkTimer.stop();
			AutonomousSequences.limelightClimber.limelight.pipeLineLED();
			AutonomousSequences.limelightFourBar.limelight.pipeLineLED();
			startedTimer = false;
			flashed = false;
		}
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
    return (Canifier.cargoBeamBreak()); // || SeriesStateMachine.forceCargoOn) && !SeriesStateMachine.forceCargoOff;
	}

	public static void printBeamBreak()
	{
		System.out.println("Cargo Beam Break: " + Robot.cargoDetection);
	}
}