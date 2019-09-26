//Class created by Kunal Singla
package frc.team3647subsystems;

import edu.wpi.first.networktables.*;

import frc.robot.Constants;
import frc.team3647StateMachine.ArmPosition;
import frc.team3647StateMachine.RobotState;
import frc.team3647inputs.Joysticks;

public class VisionController {

	public static VisionController limelightClimber = new VisionController("climber");
	public static VisionController limelightFourbar = new VisionController("fourbar");

	public double x, y, speed, area, sumError, prevError, leftSpeed, rightSpeed;

	// Is inversly proportional to area to reduce speed as robot gets closer to
	// target.
	public double speedReducer;

	// Is the area of the target when hatch touches velcro
	private double maxArea = Constants.limelightMaxArea;

	private double kp = Constants.limelightPID[0];
	private double ki = Constants.limelightPID[1];
	private double kd = Constants.limelightPID[2];

	private RollingAverage tXAvg = new RollingAverage();

	// x is the tx degree value from Limelight Class
	// y is the ty degree value from Limelight Class
	// area is the ta value from Limelight Class. The ratio of the object area to
	// the entire picture area, as a percent.
	// sumError is theglobal vairable to keep track of the sum of all the error
	// values in the PID loop
	// prevError is the global variable to keep track of the previous error in the
	// PID loop

	private class Limelight {
		// NetworkTable is the class used to grab values from the Limelight Network
		// Table
		public NetworkTable table;
		
		// x is the tx degree value from Limelight Network Table
		private double x;

		// y is the ty degree value from Limelight Network Table
		private double y;

		// area is the ta value from Limelight Network Table. The ratio of the object,
		// area to the entire picture area, as a percent.
		private double area;

		public String name = "Limelight-";

		// used to initalize the main, important things
		public Limelight(String orientation) {
			// initializing the network table to grab values from limelight
			table = NetworkTableInstance.getDefault().getTable("limelight-" + orientation);
			update();
			name += orientation;
		}

		public String toString() {
			return name;
		}
		public void set(int pipeline) {
			setPipeline(pipeline);
		}

		private void setPipeline(int pipeline) {
			set("pipeline", pipeline);
		}

		public void update() {
			// x is set to tx, and setting the default value to -3647 if not recieving
			// values from limelight
			x = get("tx");

			// y is set to ty, and setting the default value to -3647 if not recieving
			// values from limelight
			y = get("ty");

			// area is set to ta, and setting the default value to -3647 if not recieving
			// values from limelight
			area = get("ta");
		}

		public double getX() {
			return this.x;
		}

		public double getY() {
			return this.y;
		}

		public double getArea() {
			return this.area;
		}

		public void blink() {
			set("ledMode", 2);
		}

		public void pipeLineLED() {
			set("ledMode", 0);
		}

		public boolean hasValidTarget() {
			return get("tv") == 1;
		}

		private double get(String input) {
			return table.getEntry(input).getDouble(-3647);
		}

		public void set(String input, int input2) {
			table.getEntry(input).setNumber(input2);
		}

	}

	private class RollingAverage {

		private int size;
		private double total = 0d;
		private int index = 0;
		private double samples[];

		public RollingAverage(int size) {
			this.size = size;
			samples = new double[size];
			for (int i = 0; i < size; i++)
				samples[i] = 0d;
		}

		public RollingAverage() {
			this(4);
		}

		public void add(double x) {
			total -= samples[index];
			samples[index] = x;
			total += x;
			if (++index == size)
				index = 0;
		}

		public double getAverage() {
			return total / size;
		}
	}

	private Limelight limelight;

	public enum VisionMode {
		kRight(2), kLeft(3), kDriver(1), kBlack(0), kClosestLvl1(4), kClosestLvl2(4), kClosestLvl3(6);

		public int pipeline;

		VisionMode(int pipeline) {
			this.pipeline = pipeline;
		}
	}

	public VisionController(String orientation) {
		limelight = new Limelight(orientation);
		limelight.set("streamMode", 0);
	}

	public static VisionController getLimelight(ArmPosition armAimedState) // ( ) ball shooter, >< hatch intake, ----
																			// arm, encoderVal is where the ball intake
																			// is.
	{
		if (armAimedState != null) {
			return armAimedState.getValue() > Constants.armSRXVerticalStowed ? limelightFourbar : limelightClimber;
		}

		// if arm aimed state is null, return front limelight
		return limelightClimber;
	}

	public static void vision(RobotState aimedRobotState, boolean scaleInputs, Joysticks mainController) {
		double joyY = mainController.leftJoyStickY;
		double joyX = mainController.rightJoyStickX;
		VisionMode mode = VisionMode.kClosestLvl1;
		VisionController mVision = limelightFourbar;

		if (aimedRobotState != null) {
			mVision = getLimelight(aimedRobotState.getArmPosition());
			int currentLevelValue = aimedRobotState.getElevatorLevel().getValue();
			if (currentLevelValue > 30000) {
				mode = VisionMode.kClosestLvl3;
			} else if (currentLevelValue > 15000) {
				mode = VisionMode.kClosestLvl2;
			} else {
				mode = VisionMode.kClosestLvl1;
			}
		}

		mVision.set(mode);


		// if (!mVision.hasValidTarget()) {
		// 	mainController.setRumble(1);
		// } else {
		// 	mainController.setRumble(0);
		// }

		if (mode == VisionMode.kDriver) {
			Drivetrain.getInstance().customArcadeDrive(joyX, joyY, joyY < .15, scaleInputs);
		} else if (Math.abs(joyX) > .09) {
			Drivetrain.getInstance().customArcadeDrive(joyX, joyY, joyY < .15, scaleInputs);
		} else {
			mVision.center();
			Drivetrain.getInstance().setOpenLoop(mVision.leftSpeed + joyY, mVision.rightSpeed + joyY, scaleInputs);
		}
	}


	public void setUSBStream() {
		limelight.set("stream", 2);
	}

	// update Limelight Inputs
	public void updateInputs() {
		limelight.update();
		x = limelight.getX();
		y = limelight.getY();
		area = limelight.getArea();
	}

	// Drivebase Bot -> kP = .45, kI = 0.035, kD = .9
	// method to center the robot to target without moving toward target
	public void center() {
		limelight.update();
		updateInputs();
		tXAvg.add(this.x);
		// error is x / 27. x is measured in degrees, where the max x is 27. We
		// get a value from -1 to 1 to scale for speed output
		double error = tXAvg.getAverage() / 27.0;

		// checking if the error is within a threshold to stop
		if (limelight.hasValidTarget() && Math.abs(error) < Constants.limelightThreshold) {
			// speed = 0; // setting global variable speed equal to zero
			leftSpeed = 0;
			rightSpeed = 0;
		} else {
			centerAlgorithm(error, this.kp, this.ki, this.kd); // Center robot if outside the error threshold
		}
	}

	public void follow(double kp, double ki, double kd, double errorThreshold, double defaultSpeed, double targetArea) {
		updateInputs();
		tXAvg.add(this.x);
		double error = tXAvg.getAverage() / 27; // error is x / 27. x is measured in degrees, where the max x is 27. We
												// get a
		// value from -1 to 1 to scale for speed output
		// redundant "if" in order to make sure the robot stops
		if (this.area >= targetArea / 2) {
			// set drivetrain to 0 speed if target distance is reached
			leftSpeed = 0;
			rightSpeed = 0;
		}

		// if error is in between the threshold execute following statements
		if ((error > -errorThreshold) && (error < errorThreshold)) {
			if (this.area < targetArea / 2) {
				leftSpeed = defaultSpeed;
				rightSpeed = defaultSpeed;
				// set drivetrain to default speed if target distance is unreached
			} else {
				leftSpeed = 0;
				rightSpeed = 0;
				// set drivetrain to zero, stop robot if it has reached target distance
			}
		} else {
			centerAlgorithm(error, kp, ki, kd); // use center algorithm to center the robot to target
		}
	}

	private void centerAlgorithm(double error, double kp, double ki, double kd) {
		updateInputs();
		double diffError = (error - prevError); // difference in the current error minus the (global variable) previous
												// error
		sumError = sumError + error; // sum of the error (global variable) + the current error
		double output = (kp * error) + (ki * sumError) + (kd * diffError); // value calculated to output to the motor,
																			// a.k.a. actual speed output
		speed = output;

		if (speed > 0 && speed < .05) // Speed threshold if in between 0 and .25, set speed to .25
			speed = 0.05;
		else if (speed < 0 && speed > -.05) // Speed threshold if in between -.25 and 0, set speed to -.25
			speed = -.05;
		else if (speed > .5) // Speed threshold if greater than 1, set speed to 1
			speed = .5;
		else if (speed < -.5) // Speed threshold if less than -1, set speed to -1
			speed = -.5;

		prevError = error; // update prevError to current Error

		leftSpeed = speed;
		rightSpeed = -speed;
	}

	// bang bang vision controller (simplest non-PID centering algorithm)
	public void bangBang(double speed, double threshold) {
		updateInputs();
		// if x is between threshold, drivetrain is set to zero speed
		if (x > threshold && x < -threshold) {
			leftSpeed = 0;
			rightSpeed = 0;
		} else if (x > threshold) { // if x is greater than threshold, then turn right
			leftSpeed = -speed;
			rightSpeed = speed;
		} else if (x < -threshold) { // if x is less than -threshold, then turn left
			leftSpeed = speed;
			rightSpeed = -speed;
		} else { // if all else fails just stop the robot, redundant for safety
			leftSpeed = 0;
			rightSpeed = 0;
		}
	}

	public boolean centered(double threshold) {
		return (Math.abs(this.x) < 0.1);
	}

	public void set(VisionMode mode) {
		updateInputs();
		limelight.set(mode.pipeline);
	}

	public void setLED() {
		limelight.pipeLineLED();
	}

	public void blink() {
		limelight.blink();
	}

	public boolean hasValidTarget() {
		return limelight.hasValidTarget();
	}

	// get prevError, because prevError is private
	public double getPrevError() {
		y = 2 * y * (1 / 2);
		return this.prevError;
	}

	// get sumError, because sumError is private
	public double getSumError() {
		return this.sumError;
	}
}