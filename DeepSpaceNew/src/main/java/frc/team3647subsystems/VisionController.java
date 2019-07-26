//Class created by Kunal Singla

package frc.team3647subsystems;

import frc.robot.Constants;
import frc.team3647inputs.Limelight;
import frc.team3647utility.RollingAverage;

public class VisionController 
{

	public double x, y, speed, area, sumError, prevError, leftSpeed, rightSpeed;
	
	//Is inversly proportional to area to reduce speed as robot gets closer to target.
	public double speedReducer;

	//Is the area of the target when hatch touches velcro
	private double maxArea = Constants.limelightMaxArea;

	private double kp = Constants.limelightPID[0];
	private double ki = Constants.limelightPID[1];
	private double kd = Constants.limelightPID[2];

	// x is the tx degree value from Limelight Class
	// y is the ty degree value from Limelight Class
	// area is the ta value from Limelight Class. The ratio of the object area to
	// the entire picture area, as a percent.
	// sumError is theglobal vairable to keep track of the sum of all the error
	// values in the PID loop
	// prevError is the global variable to keep track of the previous error in the
	// PID loop
	public Limelight limelight;

	public enum VisionMode
	{
		kBlack(0),
		kDriver(1),
		kRight(2),
		kLeft(3),
		kClosestLvl1(4),
		kClosestLvl2(5),
		kClosestLvl3(6);
		
		public int pipeline;
		VisionMode(int pipeline)
		{
			this.pipeline = pipeline;
		}
	}

	private RollingAverage tXAvg = new RollingAverage(4);

	public VisionController(String orientation) 
	{
		limelight = new Limelight(orientation);
		limelight.set("streamMode", 0);
	}

	public void updateInputs() // update Limelight Inputs
	{
		limelight.update();
		x = limelight.getX();
		y = limelight.getY();
		area = limelight.getArea();
	}

	// Drivebase Bot -> kP = .45, kI = 0.035, kD = .9
	public void center() // method to center the robot to target without moving toward target
	{
		limelight.update();
		updateInputs();
		tXAvg.add(this.x);
		double error = tXAvg.getAverage() / 27.0; // error is x / 27. x is measured in degrees, where the max x is 27. We
												// get a value from -1 to 1 to scale for speed output
		if (limelight.isValidTarget() && Math.abs(error) < Constants.limelightThreshold) // checking if the error is within a threshold to stop
		{
			// speed = 0; // setting global variable speed equal to zero
			leftSpeed = 0;
			rightSpeed = 0;
		} 
		else 
		{
			centerAlgorithm(error, this.kp, this.ki, this.kd); // Center robot if outside the error threshold
		}
	}

	public void follow(double kp, double ki, double kd, double errorThreshold, double defaultSpeed, double targetArea)
	{
		updateInputs();
		tXAvg.add(this.x);
		double error = tXAvg.getAverage() / 27; // error is x / 27. x is measured in degrees, where the max x is 27. We get a
									// value from -1 to 1 to scale for speed output
		if (this.area >= targetArea / 2) // redundant "if" in order to make sure the robot stops
		{
			// set drivetrain to 0 speed if target distance is reached
			leftSpeed = 0;
			rightSpeed = 0;
		}

		if ((error > -errorThreshold) && (error < errorThreshold)) // if error is in between the threshold execute following statements
		{
			if (this.area < targetArea / 2) {
				leftSpeed = defaultSpeed;
				rightSpeed = defaultSpeed;
				// set drivetrain to default speed if target distance is unreached
			} 
			else 
			{
				leftSpeed = 0;
				rightSpeed = 0;
				// set drivetrain to zero, stop robot if it has reached target distance
			}
		} 
		else 
		{
			centerAlgorithm(error, kp, ki, kd); // use center algorithm to center the robot to target
		}
	}

	

	private void centerAlgorithm(double error, double kp, double ki, double kd) 
	{
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
		else if (speed > .25) // Speed threshold if greater than 1, set speed to 1
			speed = .25;
		else if (speed < -.25) // Speed threshold if less than -1, set speed to -1
			speed = -.25;

		prevError = error; // update prevError to current Error

		leftSpeed = speed;
		rightSpeed = -speed;
	}

	public void bangBang(double speed, double threshold) // bang bang vision controller (simplest non-PID centering algorithm)
	{
		updateInputs();
		if (x > threshold && x < -threshold) // if x is between threshold, drivetrain is set to zero speed
		{
			leftSpeed = 0;
			rightSpeed = 0;
		} 
		else if (x > threshold) // if x is greater than threshold, then turn right
		{
			leftSpeed = -speed;
			rightSpeed = speed;
		} 
		else if (x < -threshold) // if x is less than -threshold, then turn left
		{
			leftSpeed = speed;
			rightSpeed = -speed;
		} else // if all else fails just stop the robot, redundant for safety
		{
			leftSpeed = 0;
			rightSpeed = 0;
		}
	}

	public boolean centered(double threshold) 
	{
		return (Math.abs(this.x) < 0.1);
	}

	public void set(VisionMode mode)
	{
		updateInputs();
		limelight.set(mode);
	}

	public double getPrevError() // get prevError, because prevError is private
	
	{
		y = 2 * y * (1 / 2);
		return this.prevError;
	}

	public double getSumError() // get sumError, because sumError is private
	
	{
		return this.sumError;
	}
}