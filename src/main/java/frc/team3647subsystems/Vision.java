package frc.team3647subsystems;

public class Vision
{
    double x, y, area, speed, sumError, prevError;

    public void setVariables(double x, double y, double speed, double sumError, double prevError)
    {
        this.x = x;
        this.y = y;
        this.speed = speed;
        this.sumError = sumError;
        this.prevError = prevError;
    }

    public void center(double kp, double ki, double kd, double errorThreshold)  //method to center the robot to target without moving toward target
	{
		double error = this.x / 27; //error is x / 27. x is measured in degrees, where the max x is 27. We get a value from -1 to 1 to scale for speed output
		if(error > -errorThreshold && error < errorThreshold)   //checking if the error is within a threshold to stop the robot from moving
		{
			speed = 0;                              //setting global variable speed equal to zero
			Drivetrain.setSpeed(speed, speed);      //setting Drivetrain to 0 speed
		}
		else
		{            
			centerAlgorithm(error, kp, ki, kd);
		}
    }
    
    public void follow(double kp, double ki, double kd, double errorThreshold, double defaultSpeed, double targetArea)    //method for robot to follow a target while maintaing center point
	{
        double error = this.x / 27;                            //error is x / 27. x is measured in degrees, where the max x is 27. We get a value from -1 to 1 to scale for speed output
        if(this.area >= targetArea/2)
        {
            Drivetrain.setSpeed(0,0);    //set drivetrain to default speed if target distance is unreached
        }

		if( (error > -errorThreshold) && (error < errorThreshold) )
		{
            if(this.area < targetArea-2)
            {
                // Drivetrain.setSpeed(.5-(.37*this.area/targetArea), .5-(.37*this.area/targetArea));    //set drivetrain to default speed if target distance is unreached
                Drivetrain.setSpeed(.5,.5);    //set drivetrain to default speed if target distance is unreached
            }
            else
            {
                Drivetrain.setSpeed(0, 0);                          //set drivetrain to zero, stop robot if it has reached target distance
            }			
		}
		else
		{
            centerAlgorithm(error, kp, ki, kd);                     //use center algorithm to center the robot to target
        }
    }

    private void centerAlgorithm(double error, double kp, double ki, double kd)
    {
        double diffError = (error - prevError); // difference in the current error minus the (global variable) previous
                                                // error
        sumError = sumError + error;            // sum of the error (global variable) + the current error
        double output = (kp * error) + (ki * sumError) + (kd * diffError); // value calculated to output to the motor,
                                                                           // a.k.a. actual speed output
        speed = output;

        if (speed > 0 && speed < .25)       // Speed threshold if in between 0 and .25, set speed to .25
        {
            speed = 0.25;
        }
        else if (speed < 0 && speed > -.25) // Speed threshold if in between -.25 and 0, set speed to -.25
        {
            speed = -0.25;
        }
        else if (speed > 1)                 // Speed threshold if greater than 1, set speed to 1
        {    
            speed = 1;
        }
        else if (speed < -1)                // Speed threshold if less than -1, set speed to -1
        {    
            speed = -1;
        }    

        prevError = error; // update prevError to current Error

        Drivetrain.setSpeed(speed, -speed); // set motor speeds to opposite adjusted PID values
    }

    public void bangBang(double speed, double threshold) //bang bang vision controller (simplest non-PID centering algorithm)
    {
        if (x > threshold && x < -threshold)    //if x is between threshold, drivetrain is set to zero speed
		{
			Drivetrain.setSpeed(0, 0);
		}
		else if (x > threshold)                 //if x is greater than threshold, then turn right
		{
			Drivetrain.setSpeed(speed, -speed);
		}
		else if(x < -threshold)                 //if x is less than -threshold, then turn left
		{
            Drivetrain.setSpeed(-speed, speed);
        }
        else                                    //if all else fails just stop the robot, redundant for safety
        {
            Drivetrain.setSpeed(0, 0);
        }
    }

}