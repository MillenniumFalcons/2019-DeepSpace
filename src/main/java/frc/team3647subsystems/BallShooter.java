package frc.team3647subsystems;

import frc.robot.*;
import frc.team3647inputs.Joysticks;

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
	
	public static void intakeCargo(double power)
	{
		setOpenLoop(-power);
	}

	public static void runSmartShooter(Joysticks controller)
	{
		if(cargoDetection() && controller.rightTrigger > .15)
			shootBall();
		else if(!cargoDetection() && controller.rightTrigger > .15)
			intakeCargo(controller.rightTrigger);
	}
	
	public static boolean cargoDetection()
	{
        if(beamBreak.get())
        {
            return false;
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