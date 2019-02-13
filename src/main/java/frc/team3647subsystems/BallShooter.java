package frc.team3647subsystems;

import frc.robot.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.*;

public class BallShooter 
{
    public static VictorSPX intakeMotor = new VictorSPX(Constants.ballShooterPin);
    public static DigitalInput beamBreak = new DigitalInput(Constants.ballShooterBeamBreakPin);
	
	public static void ballShooterinitialization()
	{
		intakeMotor.setInverted(false);
	}
	
	public static void setOpenLoop(double speed)
	{
		intakeMotor.set(ControlMode.PercentOutput, speed);
    }
    
    	public static void stopMotor()
	{
		setOpenLoop(0);
	}

	public static void shootBall()
	{
		setOpenLoop(1);
	}
	
	public static void intakeCargo()
	{
		setOpenLoop(-0.75);
	}

	public static void runShooter(boolean joyValue)
	{
		if(joyValue)
		{
			shootBall();
		}
		else
		{
			stopMotor();
		}
	}
	
	public static boolean cargoDetection()
	{
        if(beamBreak.get())
        {
            return true;
        }
        else
        {
            return false;
        }
	}

	public static void printBeamBreak()
	{
		System.out.println("Cargo Beam Break: " + cargoDetection());
	}
}