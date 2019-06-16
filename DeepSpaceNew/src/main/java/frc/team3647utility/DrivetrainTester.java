package frc.team3647utility;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3647subsystems.Drivetrain;

public class DrivetrainTester {
    private Drivetrain mDrivetrain;
    
    private double prevLeftVelocity, prevRightVelocity;
	private static double maxLeftVelocity, maxRightVelocity;

	private double prevLeftAccel, prevRightAccel;
	private double maxLeftAccel, maxRightAccel;

	// private static double prevRightJerk, prevLeftJerk;
	private double maxRightJerk, maxLeftJerk;
	private Timer maxVelAccelTimer;

    public DrivetrainTester (Drivetrain dt) {
        mDrivetrain = dt;
		// Velocity variables initialization (and reset)
		// Used to get the acceleration of left Motor with the current velocity.
		// (derivative)
		prevLeftVelocity = 0;
		// Used to get the acceleration of right Motor with the current velocity.
		// (derivative)
		prevRightVelocity = 0;

		// Will store the max retrieved velocity at every point while the velAccel()
		// method runs
		maxLeftVelocity = 0;
		maxRightVelocity = 0;

		// Acceleration variables initialization (and reset)
		// prev Accel isn't currently used
		prevLeftAccel = 0;
		prevRightAccel = 0;

		// Will store the max calculated acceleration at every point while the
		// velAccel() method runs
		maxLeftAccel = 0;
		maxRightAccel = 0;

		// prevLeftJerk = 0;
		// prevRightJerk = 0;

		maxRightJerk = 0;
		maxLeftJerk = 0;
    }

    // -- -- -- Methods for testing max velocity and acceleration -- -- -- //
	// // Resets all values needed for testing max velocity and acceleration
	// /**
	// * This method is required to run before running {@link #VelAccel()} , it
	// resets all the values, the timer object and the encoders
	// *
	// */


	public void initializeVelAccel() {
		maxVelAccelTimer = new Timer();
		// Resets timer to make the programs run for 2sec
		maxVelAccelTimer.reset();
		maxVelAccelTimer.start();
		mDrivetrain.resetEncoders();

		// Velocity variables initialization (and reset)
		// Used to get the acceleration of left Motor with the current velocity.
		// (derivative)
		prevLeftVelocity = 0;
		// Used to get the acceleration of right Motor with the current velocity.
		// (derivative)
		prevRightVelocity = 0;

		// Will store the max retrieved velocity at every point while the velAccel()
		// method runs
		maxLeftVelocity = 0;
		maxRightVelocity = 0;

		// Acceleration variables initialization (and reset)
		// prev Accel isn't currently used
		prevLeftAccel = 0;
		prevRightAccel = 0;

		// Will store the max calculated acceleration at every point while the
		// velAccel() method runs
		maxLeftAccel = 0;
		maxRightAccel = 0;

		// prevLeftJerk = 0;
		// prevRightJerk = 0;

		maxRightJerk = 0;
		maxLeftJerk = 0;
	}

	// Puts velocity on the smart dashboard
	public void initializeSmartDashboardVelAccel() {
		SmartDashboard.putNumber("lMaxVelocity", 0);
		SmartDashboard.putNumber("lCurrentVelocity", 0);
		SmartDashboard.putNumber("rMaxVelocity", 0);
		SmartDashboard.putNumber("rCurrentVelocity", 0);

		SmartDashboard.putNumber("lMaxAccel", 0);
		SmartDashboard.putNumber("lCurrentAccel", 0);
		SmartDashboard.putNumber("rMaxAccel", 0);
		SmartDashboard.putNumber("rCurrentAccel", 0);
		SmartDashboard.putNumber("encoderValue", 0);

		// Displays current and max acceleration values on smartdashboard
		SmartDashboard.putNumber("lMaxJerk", 0);
		SmartDashboard.putNumber("lCurrentJerk", 0);
		SmartDashboard.putNumber("rMaxJerk", 0);
		SmartDashboard.putNumber("rCurrentJerk", 0);
	}

	// Converting encoder ticks per 100ms to meters per second (m/s)
	// Takes sensors encoder ticks per .1second (100ms) and wheel radius to get
	// velocity for left and right motors
	private double encoderToMpS(double encoderVal, double wheelRadius) {
		// Makes the encoder absolute value in case the motors are backwards or not set
		// inverted
		return Math.abs(encoderVal) / .1 / 4096.0 * .0254 * wheelRadius * Math.PI * 2;
	}

	// Running the motors at full speed in order to test max velocity and
	// acceleration
	private void runMotorsMax() {
		mDrivetrain.setOpenLoop(1, 1);
	}

	/**
	 * A method that calculates max acceleration and velocity for robot, requires
	 * running {@link #initializeVelAccel()} , as well
	 * <p>
	 * The method uses getSelectedSensorVelocity(0) an SRX method that gives encoder
	 * ticks / 100ms, it then converts to meters/second and uses conditional to
	 * determine max velocity.
	 * </p>
	 * 
	 * <p>
	 * To get max acceleration the method subtracts previous velocity from current
	 * velocity and divides by .02s, the time between every report of velocity from
	 * the motor controller. It skips the first 20ms when calculating max
	 * acceleration in order to ignore the first "jolt" of the robot and get more
	 * accurate values.
	 * </p>
	 * 
	 */
	public void velAccel() {
		// Runs motors in full power
		runMotorsMax();

		// Store current velocity for left motor in meters/second, 3 is wheel radius in
		// inches
		double leftMpS = encoderToMpS(mDrivetrain.getLeftEncoderValue(), 3);
		// Store current velocity for right motor in meters/second, 3 is wheel radius in
		// inches
		double rightMpS = encoderToMpS(mDrivetrain.getRightEncoderValue(), 3);

		// Compare current velocity to max velocity recoreded
		if (leftMpS > maxLeftVelocity) {
			// If current left velocity is bigger than max, set max to current
			maxLeftVelocity = leftMpS;
		}
		if (rightMpS > maxRightVelocity) {
			// Same for right motor
			maxRightVelocity = rightMpS;
		}

		// Initializes the left acceleration and right acceleration variables as 0
		double leftAccel = 0;
		double rightAccel = 0;

		double rightJerk = 0;
		double leftJerk = 0;

		/*
		 * In order to ignore the first 20ms of acceleration, the "jolt" at the start
		 * and get more accurate acceleration the program checks that the timer is after
		 * 20ms and then calculates acceleration
		 */
		if (maxVelAccelTimer.get() > .02) {
			// Gets left motor acceleration by subtracting previous velocity from current
			// and divide by .02s (the time between every report for velocity)
			leftAccel = (leftMpS - prevLeftVelocity) / .02;
			// Same for right motor
			rightAccel = (rightMpS - prevRightVelocity) / .02;

			leftJerk = (leftAccel - prevLeftAccel) / .02;
			rightJerk = (rightAccel - prevRightAccel) / .02;
			// Compare previous acceleration to current acceleration
			if (leftAccel > maxLeftAccel) {
				// If current bigger than max, set max to current
				maxLeftAccel = leftAccel;
			}
			if (rightAccel > maxRightAccel) {
				// If current bigger than max, set max to current
				maxRightAccel = rightAccel;
			}

			if (leftJerk > maxLeftJerk) {
				maxLeftJerk = leftJerk;
			}
			if (rightJerk > maxRightJerk) {
				maxRightJerk = rightJerk;
			}

		}
		// Displays current and max velocity values on smartdashboard
		SmartDashboard.putNumber("lMaxVelocity", maxLeftVelocity);
		SmartDashboard.putNumber("lCurrentVelocity", leftMpS);
		SmartDashboard.putNumber("rMaxVelocity", maxRightVelocity);
		SmartDashboard.putNumber("rCurrentVelocity", rightMpS);

		// Displays current and max acceleration values on smartdashboard
		SmartDashboard.putNumber("lMaxAccel", maxLeftAccel);
		SmartDashboard.putNumber("lCurrentAccel", leftAccel);
		SmartDashboard.putNumber("rMaxAccel", maxRightAccel);
		SmartDashboard.putNumber("rCurrentAccel", rightAccel);

		// Displays current and max acceleration values on smartdashboard
		SmartDashboard.putNumber("lMaxJerk", maxLeftJerk);
		SmartDashboard.putNumber("lCurrentJerk", leftJerk);
		SmartDashboard.putNumber("rMaxJerk", maxRightJerk);
		SmartDashboard.putNumber("rCurrentJerk", rightJerk);

		// Sets previous velocity to current velocity before method ends
		prevLeftVelocity = leftMpS;
		prevRightVelocity = rightMpS;

		// Sets previous acceleraion to current acceleraion before method ends
		prevLeftAccel = leftAccel;
		prevRightAccel = rightAccel;
	}

	// Getters for max velocity, left and right
	public double getMaxLeftVelocity() {
		return maxLeftVelocity;
	}

	public double getMaxRightVelocity() {
		return maxRightVelocity;
	}

	// Getters for max acceleration, left and right
	public double getMaxLeftAccel() {
		return maxLeftAccel;
	}

	public double getMaxRightAccel() {
		return maxRightAccel;
	}

	// // -- -- -- End of methods to test max velocity and acceleration -- -- -- //

}