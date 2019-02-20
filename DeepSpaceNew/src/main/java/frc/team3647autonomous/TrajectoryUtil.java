package frc.team3647autonomous;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

public class TrajectoryUtil 
{

    public static Trajectory reversePath(Trajectory originalTrajectory)
    {
        ArrayList<Segment> segments = new ArrayList<>(Arrays.asList(originalTrajectory.segments));
        Collections.reverse(segments);

        double distance = segments.get(0).position;

        return new Trajectory(segments.stream()
                .map(segment -> new Segment(segment.dt, segment.x, segment.y, distance - segment.position, -segment.velocity, -segment.acceleration, -segment.jerk, segment.heading))
                .toArray(Segment[]::new));
    }

    public static Trajectory getTrajectoryFromName(String trajectoryName)
    {
        File trajectoryFile = new File("/home/lvuser/paths/" + trajectoryName + "_source.csv");

        Trajectory trajectory; // = trajectoryFile.exists() ? Pathfinder.readFromFile(trajectoryFile) : null;
        trajectory = Pathfinder.readFromCSV(trajectoryFile);
        if(trajectory == null)
        {
            System.out.println("nO fiLE");
            // trajectoryFile = new File("C:\\Users\\brian\\OneDrive\\Projects\\FRC_2018_Offseason\\PathPlanner\\Trajectories\\" + trajectoryName + "\\" + trajectoryName + "_source_detailed.traj");
            // trajectory = trajectoryFile.exists() ? Pathfinder.readFromFile(trajectoryFile): null;
        }
        else
            System.out.println("Raed form csv succcseful");

        return trajectory;
    }
}
