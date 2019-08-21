package frc.team3647subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import frc.robot.Constants;
import frc.team3647StateMachine.SeriesStateMachine;
import frc.team3647inputs.Joysticks;
import frc.team3647subsystems.VisionController.VisionMode;

public class Drivetrain {
	private WPI_TalonSRX leftSRX;
	private WPI_TalonSRX rightSRX;

	private VictorSPX leftSPX1;
	private VictorSPX leftSPX2;

	private VictorSPX rightSPX2;
	private VictorSPX rightSPX1;

	private int leftEncoderValue;
	private int rightEncoderValue;

	private int leftVelocity;
	private int rightVelocity;

	private DifferentialDrive drive;

	public boolean initialized;

	private static Drivetrain INSTANCE = new Drivetrain();

	private Drivetrain() {
		leftSRX = new WPI_TalonSRX(Constants.leftMasterPin);
		leftSPX1 = new VictorSPX(Constants.leftSlave1Pin);
		leftSPX2 = new VictorSPX(Constants.leftSlave2Pin);

		rightSRX = new WPI_TalonSRX(Constants.rightMasterPin);
		rightSPX1 = new VictorSPX(Constants.rightSlave1Pin);
		rightSPX2 = new VictorSPX(Constants.rightSlave2Pin);

		drive = new DifferentialDrive(leftSRX, rightSRX);
		initialized = false;
	}

	public static Drivetrain getInstance() {
		return INSTANCE;
	}

	public void init() {
		// Set up followers
		setFollowers();

		configLeftSRX();
		configRightSRX();

		// leftSRX.setExpiration(Constants.expirationTimeSRX);
		// rightSRX.setExpiration(Constants.expirationTimeSRX);
		// drive.setExpiration(Constants.expirationTimeSRX);
		drive.setSafetyEnabled(false);
		leftSRX.setSafetyEnabled(false);
		rightSRX.setSafetyEnabled(false);

		selectPIDF(0, Constants.rightVelocityPIDF, Constants.leftVelocityPIDF);

		setToBrake();

		drive.setRightSideInverted(false);
		drive.setDeadband(.09);

		initialized = true;
	}

	// --- SETUP METHODS FOR MOTORS --- //

	private void setFollowers() {
		leftSPX1.follow(leftSRX);
		leftSPX2.follow(leftSRX);

		rightSPX1.follow(rightSRX);
		rightSPX2.follow(rightSRX);
	}

	private void configLeftSRX() {
		// Config left side PID settings
		leftSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs);
		leftSRX.setSensorPhase(true);
		leftSRX.configNominalOutputForward(0, Constants.kTimeoutMs);
		leftSRX.configNominalOutputReverse(0, Constants.kTimeoutMs);
		leftSRX.configPeakOutputForward(1, Constants.kTimeoutMs);
		leftSRX.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		// Config current limiting
		leftSRX.enableCurrentLimit(true);
		leftSRX.configPeakCurrentLimit(Constants.drivePeakCurrent);
		leftSRX.configPeakCurrentDuration(Constants.drivePeakCurrentDuration);

		leftSRX.configContinuousCurrentLimit(Constants.driveContinousCurrent);
		leftSRX.setName("Drivetrain", "leftSRX");
	}

	private void configRightSRX() {
		// Config right side PID settings
		rightSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.driveSlotIdx,
				Constants.kTimeoutMs);
		rightSRX.setSensorPhase(true);
		rightSRX.configNominalOutputForward(0, Constants.kTimeoutMs);
		rightSRX.configNominalOutputReverse(0, Constants.kTimeoutMs);
		rightSRX.configPeakOutputForward(1, Constants.kTimeoutMs);
		rightSRX.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		// Config current limiting
		rightSRX.enableCurrentLimit(true);
		rightSRX.configPeakCurrentLimit(Constants.drivePeakCurrent);
		rightSRX.configPeakCurrentDuration(Constants.drivePeakCurrentDuration);
		rightSRX.configContinuousCurrentLimit(Constants.driveContinousCurrent);

		rightSRX.setInverted(true);
		rightSPX1.setInverted(true);
		rightSPX2.setInverted(true);

		rightSRX.setName("Drivetrain", "rightSRX");
	}

	// --- --- --- --- --- --- --- --- --- --- --- //

	public void selectPIDF(int slot, double[] right, double[] left) {
		// PID SLOT
		rightSRX.selectProfileSlot(slot, 0);
		leftSRX.selectProfileSlot(slot, 0);

		// PID
		rightSRX.config_kP(slot, right[0], Constants.kTimeoutMs);
		rightSRX.config_kI(slot, right[1], Constants.kTimeoutMs);
		rightSRX.config_kD(slot, right[2], Constants.kTimeoutMs);
		rightSRX.config_kF(slot, right[3], Constants.kTimeoutMs);

		leftSRX.config_kP(slot, left[0], Constants.kTimeoutMs);
		leftSRX.config_kI(slot, left[1], Constants.kTimeoutMs);
		leftSRX.config_kD(slot, left[2], Constants.kTimeoutMs);
		leftSRX.config_kF(slot, left[3], Constants.kTimeoutMs);
	}

	
	public void driveVisionTeleop(Joysticks mainController, SeriesStateMachine stateMachine, boolean scaleInputs) {
		if (mainController.rightBumper) {
			VisionController.vision(stateMachine.getAimedRobotState(), scaleInputs, mainController);
		} else {
			mainController.setRumble(0);
			VisionController.limelightClimber.set(VisionMode.kBlack);
			VisionController.limelightFourbar.set(VisionMode.kBlack);
			customArcadeDrive(mainController.rightJoyStickX, mainController.leftJoyStickY * .6,
					mainController.leftJoyStickY < .4,
					(scaleInputs || mainController.rightJoyStickPress));
		}
	}

	/**
	 * Method to control robot
	 * 
	 * @param xValue joystick x value
	 * @param yValue joystick y value
	 * @param gyro   gyro object
	 */
	public void customArcadeDrive(double xValue, double yValue, boolean quickTurn, boolean scaleInputs) {
		if (scaleInputs) {
			xValue *= .65;
			yValue *= .6;
		}

		drive.curvatureDrive(yValue, xValue, quickTurn);
		// drive.arcadeDrive(yValue, xValue);
	}


	public void arcadeDrive(double throttle, double turn, boolean scaleInputs) {
		throttle = limit(throttle);
		turn = limit(turn);
		double leftMotorOutput;
		double rightMotorOutput;

		double maxInput = Math.copySign(Math.max(Math.abs(throttle), Math.abs(turn)), throttle);

		if (throttle >= 0.0) {
			// First quadrant, else second quadrant
			if (turn >= 0.0) {
				leftMotorOutput = maxInput;
				rightMotorOutput = throttle - turn;
			} else {
				leftMotorOutput = throttle + turn;
				rightMotorOutput = maxInput;
			}
		} else {
			// Third quadrant, else fourth quadrant
			if (turn >= 0.0) {
				leftMotorOutput = throttle + turn;
				rightMotorOutput = maxInput;
			} else {
				leftMotorOutput = maxInput;
				rightMotorOutput = throttle - turn;
			}
		}

		if (scaleInputs) {
			leftMotorOutput = limit(leftMotorOutput) * .6;
			rightMotorOutput = limit(rightMotorOutput) * .6;
		}
		leftSRX.set(limit(leftMotorOutput));
		rightSRX.set(limit(rightMotorOutput));

	}

	private double limit(double value) {
		if (value > 1) {
			value = 1;
		} else if (value < -1) {
			value = -1;
		}
		return value;
	}


	public void setOpenLoop(double lOutput, double rOutput) {
		rightSRX.set(ControlMode.PercentOutput, rOutput);
		leftSRX.set(ControlMode.PercentOutput, lOutput);
	}

	public void setOpenLoop(double lOutput, double rOutput, boolean scaleInputs) {
		if (scaleInputs) {
			rOutput *= .6;
			lOutput *= .6;
		}

		rightSRX.set(ControlMode.PercentOutput, rOutput);
		leftSRX.set(ControlMode.PercentOutput, lOutput);
	}

	public void stop() {
		rightSRX.stopMotor();
		leftSRX.stopMotor();
	}

	public void velocityDrive(double xValue, double yValue) {
		selectPIDF(Constants.velocitySlotIdx, Constants.rightVelocityPIDF, Constants.leftVelocityPIDF);
		double threshold = 0.09;
		if (yValue != 0 && Math.abs(xValue) < threshold) {
			setVelocity(yValue, yValue);
		} else if (yValue == 0 && Math.abs(xValue) < threshold) {
			resetEncoders();
			stop();
		} else {
			curvatureDrive(xValue, yValue);
		}
	}
									//puts x         //puts y
	private void curvatureDrive(double speed, double rotation) {
		try {
			drive.curvatureDrive(speed, rotation, rotation < .15); // curvature drive from WPILIB libraries.
		} catch (NullPointerException e) {
			System.err.println(e);
			System.err.println("differential drive not initialized\nCreating new DifferentialDrive object");
			drive = new DifferentialDrive(leftSRX, rightSRX);
		}
	}

	// USED BY AUTO FOR SOME REASON
	public void setVelocity(double lSpeed, double rSpeed) {
		double targetVelocityRight = rSpeed * Constants.velocityConstant;
		double targetVelocityLeft = lSpeed * Constants.velocityConstant;

		rightSRX.set(ControlMode.Velocity, targetVelocityRight);
		leftSRX.set(ControlMode.Velocity, targetVelocityLeft);
	}

	public void setRawVelocity(double lSpeed, double rSpeed) {
		leftSRX.set(ControlMode.Velocity, lSpeed);
		rightSRX.set(ControlMode.Velocity, rSpeed);
	}

	public void setAutoVelocity(double leftDriveSignal, double rightDriveSignal) {
		rightSRX.set(ControlMode.Velocity, rightDriveSignal);
		leftSRX.set(ControlMode.Velocity, leftDriveSignal);
	}

	public void setToBrake() {
		leftSRX.setNeutralMode(NeutralMode.Brake);
		rightSRX.setNeutralMode(NeutralMode.Brake);
		leftSPX1.setNeutralMode(NeutralMode.Brake);
		leftSPX2.setNeutralMode(NeutralMode.Brake);
		rightSPX1.setNeutralMode(NeutralMode.Brake);
		rightSPX2.setNeutralMode(NeutralMode.Brake);
	}

	public void setToCoast() {
		leftSRX.setNeutralMode(NeutralMode.Coast);
		rightSRX.setNeutralMode(NeutralMode.Coast);
		leftSPX1.setNeutralMode(NeutralMode.Coast);
		leftSPX2.setNeutralMode(NeutralMode.Coast);
		rightSPX1.setNeutralMode(NeutralMode.Coast);
		rightSPX2.setNeutralMode(NeutralMode.Coast);
	}

	public void updateEncoders() {
		leftEncoderValue = leftSRX.getSelectedSensorPosition();
		rightEncoderValue = rightSRX.getSelectedSensorPosition();

		leftVelocity = leftSRX.getSelectedSensorVelocity();
		rightVelocity = rightSRX.getSelectedSensorVelocity();
	}

	public void resetEncoders() {
		resetLeftEncoderValue();
		resetRightEncoder();
	}

	public void setEncoders(int leftVal, int rightVal) {
		setLeftEncoderValue(leftVal);
		setRightEncoderValue(rightVal);
	}

	public void setLeftEncoderValue(int val) {
		leftSRX.setSelectedSensorPosition(val);
	}

	public void resetLeftEncoderValue() {
		setLeftEncoderValue(0);
	}

	public int getLeftEncoderValue() {
		return leftEncoderValue;
	}

	public void setRightEncoderValue(int val) {
		rightSRX.setSelectedSensorPosition(val);
	}

	public void resetRightEncoder() {
		setRightEncoderValue(0);
	}
	
	public int getRightEncoderValue() {
		return rightEncoderValue;
	}

	public int getLeftVelocity() {
		return leftVelocity;
	}

	public int getRightVelocity() {
		return rightVelocity;
	}

	public void testDrivetrainCurrent() {
		System.out.println("Left Motor Current: " + leftSRX.getOutputCurrent());
		System.out.println("Right Motor Current:" + rightSRX.getOutputCurrent());
	}

	public void printEncoders() {
		System.out.println("Left Encoder: " + leftSRX.getSelectedSensorPosition());
		System.out.println("Right Encoder:" + rightSRX.getSelectedSensorPosition());
	}

	public void printVelocity() {
		System.out.println("Left Vel: " + leftSRX.getSelectedSensorVelocity());
		System.out.println("Right Vel:" + rightSRX.getSelectedSensorVelocity());
	}

	public int prevVelR = 0, prevVelL = 0;

	public void printAccel() {
		int currentVelR = rightSRX.getSelectedSensorVelocity();
		int currentVelL = leftSRX.getSelectedSensorVelocity();

		System.out.println("Left accel: " + (currentVelL - prevVelL) / .02);
		System.out.println("Right Vel:" + (currentVelR - prevVelR) / .02);

		prevVelR = rightSRX.getSelectedSensorVelocity();
		prevVelL = leftSRX.getSelectedSensorVelocity();
	}

	public void printVelError() {
		int velErrorR = rightSRX.getClosedLoopError();
		int velErrorL = leftSRX.getClosedLoopError();
		System.out.println("Right Vel Error: " + velErrorR);
		System.out.println("Left Vel Error: " + velErrorL);
	}

	
}
