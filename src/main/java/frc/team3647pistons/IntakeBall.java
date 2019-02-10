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
	public IntakeBallState currentState, aimedState;
	public enum IntakeBallState
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

	public void setAimedState(IntakeBallState aimedState)
	{
		this.aimedState = aimedState;
	}

	public IntakeBallState getAimedState()
	{
		return this.aimedState;
	}

	public IntakeBallState getCurrentState()
	{
		return this.currentState;
	}
}
