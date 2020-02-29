package org.team3128.compbot.autonomous;

import java.util.ArrayList;

import org.team3128.common.control.trajectory.Trajectory;
import org.team3128.common.control.trajectory.TrajectoryGenerator;
import org.team3128.common.drive.Drive;
import org.team3128.common.utility.Log;
import org.team3128.common.utility.math.Pose2D;
import org.team3128.testbench.subsystems.Constants;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class CmdAutoTrajectory extends CommandGroup {

    public Trajectory trajectory;

    private double speed, acceleration, timeInitial, timeExpected, tiemCurrent, timeoutMs;

    public ArrayList<Pose2D> waypoints = new ArrayList<Pose2D>();

    public CmdAutoTrajectory(Drive drive, double speed, double acceleration, double timeoutMs, Pose2D Pose2D) {
        this.speed = speed;
        this.acceleration = acceleration;
        this.drive = drive;
        this.timeoutMs = timeoutMs;

        for (Pose2D)

    }
    @Override protected void initialize() {
        trajectory = TrajectoryGenerator.generateTrajectory(waypoints, new ArrayList<Pose2D>, speed * Constants.inchesToMeters, acceleration, false);
        drive.setAutoTrajetory(trajectory, false);
        drive.startTrajectory();
        timeExpected = trajectory.getTotalTimeSeconds;
        timeInitial = Timer.getFPGATimestamp();
    }

    @Override protected void execute() {

    }

    @Override protected synchronized boolean isFinished() {
        currentTime = Timer.getFPGATimestamp();
        return (Timer.getFPGATimestamp() - timeInitial >= timeExpected);
    }

    @Override
    protected void interrupted() {
        Log.info("CmdAutoTrajectory", "Interrupted.");
    }
}