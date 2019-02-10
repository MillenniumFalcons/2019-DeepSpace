package frc.team3647pistons;

import frc.robot.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.*;

public class IntakeBall 
{
	private Solenoid piston = new Solenoid(Constants.ballIntakeFC);
	private VictorSPX ballSPX = new VictorSPX(Constants.ballMotorPin);
	private DigitalInput reed = new DigitalInput(8);
	public BallState currentState, aimedState;
	public enum BallState
	{
		RetractedRollingForward,
		RetractedRollingBackwards,
		RetractedNotRolling,
		ExtractedRollingForward,
		ExtractedRollingBackward,
		ExtractedNotRolling
	}
	
	public void spinRoller(double speedInput)
	{
		ballSPX.set(ControlMode.PercentOutput, speedInput);
	}

	public void openIntake()
	{
		piston.set(true);
	}
	
	public void closeIntake()
	{
		piston.set(false);
	}
	
	public void runIntake(boolean joyValue)
	{
		if(joyValue)
		{
			openIntake();
		}
		else
		{
			closeIntake();
		}
	}
}
