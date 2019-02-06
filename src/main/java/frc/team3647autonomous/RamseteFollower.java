package frc.team3647autonomous;

import frc.team3647subsystems.*;
import frc.team3647utility.*;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;
import java.io.File;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class RamseteFollower
{
    Trajectory sourceTrajectory;
    Odometry odo = new Odometry();
    Drivetrain mDrivetrain = new Drivetrain();
    int pointNum;

    public RamseteFollower(String path, boolean backwards)
    {
        sourceTrajectory = Pathfinder.readFromCSV(new File("/home/lvuser/paths/" + path + "_source_Jaci.csv")); //Load path following file from filepath
        System.out.println("Path has been successfully loaded!");                                               //Indicate Loading of Path was Successful
        odo.odometryInit(backwards);                                                                                     //Start the periodic notifier event
        pointNum = 0;                                                                                           //Start at the beginning of a trajectory
        System.out.println("Starting Position: " + sourceTrajectory.get(0).x + " " + sourceTrajectory.get(0).y + " " +sourceTrajectory.get(0).heading);
        odo.setOdometry(sourceTrajectory.get(0).x, sourceTrajectory.get(0).y, sourceTrajectory.get(0).heading);
    }

    public void runPath()
    {
        Segment currentSegment = sourceTrajectory.get(pointNum);
        double linVel = adjustedLinVel(currentSegment.x, currentSegment.y, currentSegment.heading, currentSegment.velocity, targetAngVel());
        double angVel = adjustedAngVel(currentSegment.x, currentSegment.y, currentSegment.heading, currentSegment.velocity, targetAngVel());
        double lOutput = ((-Units.inchesToMeters(Constants.kWheelBase) * angVel) / 2 + linVel) * (1/Units.feetToMeters(Constants.kMaxVelocity)); //calculate velocity in m/s then convert to scale of -1 to 1
        double rOutput = ((+Units.inchesToMeters(Constants.kWheelBase) * angVel) / 2 + linVel) * (1/Units.feetToMeters(Constants.kMaxVelocity)); //v = ω*r
        SmartDashboard.putNumber("Target Velocity", linVel);            //Diplay Target Velocity in Dashboard
        SmartDashboard.putNumber("Target Angular Velocity", angVel);    //Display Target Angular Velocity in Dashboard
        SmartDashboard.putNumber("lOutput", lOutput);                   //Display adjusted left speed in Dashboard
        SmartDashboard.putNumber("rOutput", rOutput);                   //Display adjusted right speed in Dashboard

        pointNum++;             //Increase pointNum as you go from one trajectory point to another
        odo.printPosition();    //Prints current position of the robot

        mDrivetrain.setVelocity(lOutput, rOutput); //Sets speed of motors to calculated adjusted speed
    }

    public void runPathBackwards()
    {
        Segment currentSegment = sourceTrajectory.get(pointNum);
        double linVel = adjustedLinVel(currentSegment.x, currentSegment.y, currentSegment.heading, currentSegment.velocity, targetAngVel());
        double angVel = adjustedAngVel(currentSegment.x, currentSegment.y, currentSegment.heading, currentSegment.velocity, targetAngVel());
        double lOutput = ((-Units.inchesToMeters(Constants.kWheelBase) * angVel) / 2 + linVel) * (1/Units.feetToMeters(Constants.kMaxVelocity)); //calculate velocity in m/s then convert to scale of -1 to 1
        double rOutput = ((+Units.inchesToMeters(Constants.kWheelBase) * angVel) / 2 + linVel) * (1/Units.feetToMeters(Constants.kMaxVelocity)); //v = ω*r
        SmartDashboard.putNumber("Target Velocity", linVel);            //Diplay Target Velocity in Dashboard
        SmartDashboard.putNumber("Target Angular Velocity", angVel);    //Display Target Angular Velocity in Dashboard
        SmartDashboard.putNumber("lOutput", lOutput);                   //Display adjusted left speed in Dashboard
        SmartDashboard.putNumber("rOutput", rOutput);                   //Display adjusted right speed in Dashboard

        pointNum++;             //Increase pointNum as you go from one trajectory point to another
        odo.printPosition();    //Prints current position of the robot

        mDrivetrain.setVelocity(-rOutput, -lOutput); //Sets speed of motors to calculated adjusted speed
    }

    /*
    public void followPath(Trajectory trajPoints, boolean backward, boolean reverse)
    {
        // Trajectory.Config configPoints = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_LOW, Constants.MPTimeStep, maxVelocity, maxAcceleration, Constants.maxJerk);
        // Trajectory trajPoints = Pathfinder.generate(points, configPoints);

        TankModifier tankModifier = new TankModifier(trajPoints);
        tankModifier.modify(Constants.wheelBase);

        if(backward)
        {
            leftTrajectory = tankModifier.getRightTrajectory();
            rightTrajectory = tankModifier.getLeftTrajectory();
        }
        else
        {
            leftTrajectory = tankModifier.getLeftTrajectory();
            rightTrajectory = tankModifier.getRightTrajectory();
        }

        finalReverse = backward;

        if(reverse)
        {
            right.setTrajectory(reverseTrajectory(rightTrajectory));
            left.setTrajectory(reverseTrajectory(leftTrajectory));
        }
        else
        {
            right.setTrajectory(rightTrajectory);
            left.setTrajectory(leftTrajectory);
        }
    }
    */
    
    public double targetAngVel()
    {
        if(pointNum < sourceTrajectory.length() - 1)
        {
            double previousTheta = sourceTrajectory.get(pointNum).heading;              //get current heading
            double nextTheta = sourceTrajectory.get(pointNum + 1).heading;              //get next heading
            return (nextTheta - previousTheta) / sourceTrajectory.get(pointNum).dt;     //calculate target angular velocity
        }
        else
        {
            return 0;
        }
    }
    
    public double kGain(double targetAngVel, double targetLinVel)
    {
        return 2 * Constants.kZeta * Math.sqrt(targetAngVel * targetAngVel + Constants.kBeta * targetLinVel * targetLinVel);
    }
    
    public double adjustedLinVel(double targetX, double targetY, double targetTheta, double targetLinVel, double targetAngVel)
    {
        double kGain = kGain(targetAngVel, targetLinVel);           //
        double xError = targetX - odo.x;                            //
        double yError = targetY - odo.y;                            //
        double thetaError = clampTheta(targetTheta - odo.theta);    //
        return targetLinVel * Math.cos(thetaError) + kGain * (Math.cos(odo.theta) * xError + Math.sin(odo.theta) * yError);
    }

    public double adjustedAngVel(double targetX, double targetY, double targetTheta, double targetLinVel, double targetAngVel)
    {
        double kGain = kGain(targetAngVel, targetLinVel);
        double xError = targetX - odo.x;
        double yError = targetY - odo.y;
        double thetaError = clampTheta(targetTheta - odo.theta);
        double sinThetaErrOverThetaErr = (Math.sin(thetaError) / thetaError);
        if (Math.abs(thetaError) < 0.00001)
        {
            sinThetaErrOverThetaErr = 1; //this is the limit as sin(x)/x approaches zero
        }
        else
        {
            sinThetaErrOverThetaErr = Math.sin(thetaError) / (thetaError);
        }
        return targetAngVel + Constants.kBeta * targetLinVel * sinThetaErrOverThetaErr * (Math.cos(odo.theta) * yError - Math.sin(odo.theta) * xError) + kGain* thetaError;
    }
    
    public double clampTheta(double theta)
    {
        return Pathfinder.d2r(Pathfinder.boundHalfDegrees(Pathfinder.r2d(theta)));
    }
    
    public boolean isFinished() 
    {
        return pointNum == sourceTrajectory.length()-1;
    }
}