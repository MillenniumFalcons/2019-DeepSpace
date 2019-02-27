//Class created by Kunal Singla

package frc.team3647subsystems;

import frc.robot.Constants;
import frc.team3647inputs.Joysticks;
import frc.team3647inputs.Limelight;

public class VisionController 
{

	public double x, y, speed, area, sumError, prevError, leftSpeed, rightSpeed;
	// x is the tx degree value from Limelight Class
	// y is the ty degree value from Limelight Class
	// area is the ta value from Limelight Class. The ratio of the object area to
	// the entire picture area, as a percent.
	// sumError is theglobal vairable to keep track of the sum of all the error
	// values in the PID loop
	// prevError is the global variable to keep track of the previous error in the
	// PID loop
	public Limelight limelight;

	public VisionController(String orientation) 
	{
		limelight = new Limelight(orientation);
	}

	public void updateInputs() // update Limelight Inputs
	{
		limelight.updateLimelight();
		x = limelight.getX();
		y = limelight.getY();
		area = limelight.getArea();
	}

	// Drivebase Bot -> kP = .45, kI = 0.035, kD = .9
	public void center(double errorThreshold) // method to center the robot to target without moving toward target
	{
		double kp = Constants.limelightPID[0];
		double ki = Constants.limelightPID[1];
		double kd = Constants.limelightPID[2];

		limelight.updateLimelight();
		updateInputs();
		double error = limelight.getX() / 27; // error is x / 27. x is measured in degrees, where the max x is 27. We
												// get a value from -1 to 1 to scale for speed output
		if (error > -errorThreshold && error < errorThreshold) // checking if the error is within a threshold to stop
		{
			speed = 0; // setting global variable speed equal to zero
			// drivesignal.setBoth(speed, speed); //setting Drivetrain to 0 speed
			leftSpeed = speed;
			rightSpeed = speed;
		} 
		else 
		{
			centerAlgorithm(error, kp, ki, kd); // Center robot if outside the error threshold
		}
	}

	public void follow(double kp, double ki, double kd, double errorThreshold, double defaultSpeed, double targetArea)
	{
		updateInputs();
		double error = this.x / 27; // error is x / 27. x is measured in degrees, where the max x is 27. We get a
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

		if (speed > 0 && speed < .25) // Speed threshold if in between 0 and .25, set speed to .25
			speed = 0.25;
		else if (speed < 0 && speed > -.25) // Speed threshold if in between -.25 and 0, set speed to -.25
			speed = -0.25;
		else if (speed > 1) // Speed threshold if greater than 1, set speed to 1
			speed = 1;
		else if (speed < -1) // Speed threshold if less than -1, set speed to -1
			speed = -1;

		prevError = error; // update prevError to current Error

		leftSpeed = speed;
		rightSpeed = -speed;
		// drivesignal.setBoth(speed, -speed); // set motor speeds to opposite adjusted
		// PID values
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
		if (Math.abs(this.x) < 0.1)
			return true;
		else
			return false;
	}

	public void disabledMode() 
	{
		limelight.pip();
		limelight.setToVison();
		limelight.ledOff();
	}

	public void visionTargetingMode() 
	{
		limelight.pip();
		limelight.ledOn();
		limelight.setToVison();
	}

	public void driverMode() 
	{
		limelight.pip();
		limelight.ledOff();
		limelight.setToVison();
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