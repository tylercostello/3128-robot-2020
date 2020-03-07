package org.team3128.compbot.autonomous; 

import org.team3128.common.control.trajectory.Trajectory;
import org.team3128.common.control.trajectory.TrajectoryGenerator;
import org.team3128.common.control.trajectory.constraint.TrajectoryConstraint;
import org.team3128.common.utility.math.Pose2D;
import org.team3128.common.utility.units.Length;
import org.team3128.compbot.subsystems.FalconDrive;
import org.team3128.common.drive.DriveSignal;
import org.team3128.compbot.subsystems.Constants;
import org.team3128.common.utility.Log;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;

public class CmdAutoTrajectory extends Command { 
    public ArrayList<Pose2D> waypoints = new ArrayList<Pose2D>();
    public Trajectory trajectory;

    private double speed, acceleration, timeExpected, timeInitial, timeCurrent, timeoutMs;
    private FalconDrive drive;
    
    public CmdAutoTrajectory(FalconDrive drive, double speed, double acceleration, double timeoutMs, Pose2D... inputWaypoints) { 

        this.speed = speed;
        this.acceleration = acceleration;
        this.drive = drive;
        this.timeoutMs = timeoutMs;

        for(Pose2D ele : inputWaypoints) {
            waypoints.add(ele);
        }
    }

    @Override protected void initialize() { 
        Log.info("1", "1");
        trajectory = TrajectoryGenerator.generateTrajectory(waypoints, new ArrayList<TrajectoryConstraint>(), 0, 0,
                speed * Constants.MechanismConstants.inchesToMeters, acceleration, false);
        Log.info("2", "2");
        drive.setAutoTrajectory(trajectory, false);
        Log.info("3", "3");
        drive.startTrajectory();
        Log.info("4", "4");
        timeExpected = trajectory.getTotalTimeSeconds();
        Log.info("5", "5");
        timeInitial = Timer.getFPGATimestamp();
        Log.info("6", "6");
    }

    @Override protected synchronized boolean isFinished() {
        Log.info("7", "7");
        timeCurrent = Timer.getFPGATimestamp();
        if ((timeCurrent - timeInitial) >= Math.min(timeExpected, timeoutMs)) {
           return true;
        } else {
           return false;
       }
    }

    @Override public void end() {
        drive.setWheelVelocity(new DriveSignal(0, 0));
    }

    @Override public void interrupted() {
        end();
        Log.info("CmdAutoTrajectory", "Interrupted.");
    }

}