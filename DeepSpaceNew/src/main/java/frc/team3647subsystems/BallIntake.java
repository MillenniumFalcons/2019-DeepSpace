package frc.team3647subsystems;

import frc.robot.Robot;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import frc.team3647utility.LazyVictorSPX;
import frc.team3647utility.Solenoid;

/**
 * A pnumatic subsystem that includes the fourbar and intake rollers
 */
public class BallIntake extends Subsystem {
	private static BallIntake INSTANCE = new BallIntake();

	public Solenoid extensionCylinder;
	public Solenoid extensionCylinder2;
	private VictorSPX intakeMotor;
	
	private BallIntake(){
		extensionCylinder = new Solenoid(Constants.ballIntakeSolinoidPin);
		extensionCylinder2 = new Solenoid(Constants.ballIntakeSolinoidPin2);
		intakeMotor = new LazyVictorSPX(Constants.ballMotorPin);
	}

	public static BallIntake getInstance(){
		return INSTANCE;
	}

	public void init(){
		intakeMotor.setInverted(false);
	}

	/**
	 * runs intake roller in, slower if detects cargo, runs ball shooter as well
	 */
	public void run(){
		if(Robot.cargoDetection){
			intake(.3);
		}else{
			intake(.8);
		}
		BallShooter.getInstance().intakeCargo(1);
	}
	
	public void setOpenLoop(double demand){
		intakeMotor.set(ControlMode.PercentOutput, demand);
	}


	public void extend(){
		extensionCylinder.set(true);
		extensionCylinder2.set(true);
	}
	
	public void retract(){
		extensionCylinder.set(false);
		extensionCylinder2.set(false);
	}


	public void intake(double power){
		setOpenLoop(power);
	}

	public void spitOut(){
		setOpenLoop(-1);
	}
}