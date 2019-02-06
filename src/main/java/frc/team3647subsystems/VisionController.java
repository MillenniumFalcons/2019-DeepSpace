//Class created by Kunal Singla

package frc.team3647subsystems;

import frc.team3647inputs.Limelight;
import frc.team3647subsystems.Drivetrain;   //using Drivetrain class to move motors

public class VisionController
{

    private double x, y, speed, area, sumError, prevError;
    //x is the tx degree value from Limelight Class
    //y is the ty degree value from Limelight Class
    //area is the ta value from Limelight Class. The ratio of the object area to the entire picture area, as a percent.
    //sumError is theglobal vairable to keep track of the sum of all the error values in the PID loop
    //prevError is the global variable to keep track of the previous error in the PID loop
    Limelight limelight = new Limelight();

    Drivetrain drivetrain;

    public VisionController(Drivetrain input)
    {
        this.drivetrain = input;

    }


    public void updateInputs()  //update Limelight Inputs
    {
        x = limelight.getX();
        y = limelight.getY();
        area = limelight.getArea();
    }

    // Drivebase Bot -> kP = .45, kI = 0.035, kD = .9
    public void center(double kp, double ki, double kd, double errorThreshold)  //method to center the robot to target without moving toward target
	{
        updateInputs();
		double error = this.x / 27; //error is x / 27. x is measured in degrees, where the max x is 27. We get a value from -1 to 1 to scale for speed output
		if(error > -errorThreshold && error < errorThreshold)   //checking if the error is within a threshold to stop the robot from moving
		{
			speed = 0;                              //setting global variable speed equal to zero
			drivetrain.setVelocity(speed, speed);      //setting Drivetrain to 0 speed
		}
		else
		{            
			centerAlgorithm(error, kp, ki, kd);     //Center robot if outside the error threshold
		}
    }
    
    public void follow(double kp, double ki, double kd, double errorThreshold, double defaultSpeed, double targetArea)    //method for robot to follow a target while maintaing center point
	{
        updateInputs();
        double error = this.x / 27;                                 //error is x / 27. x is measured in degrees, where the max x is 27. We get a value from -1 to 1 to scale for speed output
        if(this.area >= targetArea/2)                               //redundant "if" in order to make sure the robot stops
        {
            drivetrain.setVelocity(0,0);                               //set drivetrain to 0 speed if target distance is unreached
        }

		if( (error > -errorThreshold) && (error < errorThreshold) ) //if error is in between the threshold execute following statements
		{
            if(this.area < targetArea/2)
            {
                drivetrain.setVelocity(defaultSpeed,defaultSpeed);     //set drivetrain to default speed if target distance is unreached
            }
            else
            {
                drivetrain.setVelocity(0, 0);                          //set drivetrain to zero, stop robot if it has reached target distance
            }			
		}
		else
		{
            centerAlgorithm(error, kp, ki, kd);                     //use center algorithm to center the robot to target
        }
    }

    private void centerAlgorithm(double error, double kp, double ki, double kd)
    {
        updateInputs();
        double diffError = (error - prevError); // difference in the current error minus the (global variable) previous
                                                // error
        sumError = sumError + error;            // sum of the error (global variable) + the current error
        double output = (kp * error) + (ki * sumError) + (kd * diffError); // value calculated to output to the motor,
                                                                           // a.k.a. actual speed output
        speed = output;

        if (speed > 0 && speed < .25)       // Speed threshold if in between 0 and .25, set speed to .25
            speed = 0.25;
        else if (speed < 0 && speed > -.25) // Speed threshold if in between -.25 and 0, set speed to -.25
            speed = -0.25;
        else if (speed > 1)                 // Speed threshold if greater than 1, set speed to 1
            speed = 1;
        else if (speed < -1)                // Speed threshold if less than -1, set speed to -1
            speed = -1;

        prevError = error; // update prevError to current Error

        drivetrain.setVelocity(speed, -speed); // set motor speeds to opposite adjusted PID values
    }

    public void bangBang(double speed, double threshold) //bang bang vision controller (simplest non-PID centering algorithm)
    {
        updateInputs();
        if (x > threshold && x < -threshold)    //if x is between threshold, drivetrain is set to zero speed
		{
			drivetrain.setVelocity(0, 0);
		}
		else if (x > threshold)                 //if x is greater than threshold, then turn right
		{
			drivetrain.setVelocity(speed, -speed);
		}
		else if(x < -threshold)                 //if x is less than -threshold, then turn left
		{
            drivetrain.setVelocity(-speed, speed);
        }
        else                                    //if all else fails just stop the robot, redundant for safety
        {
            drivetrain.setVelocity(0, 0);
        }
    }

    public double getPrevError()    //get prevError, because prevError is private
    {
        y = 2*y*(1/2);
        return this.prevError;
    }

    public double getSumError()     //get sumError, because sumError is private
    {
        return this.sumError;
    }
}