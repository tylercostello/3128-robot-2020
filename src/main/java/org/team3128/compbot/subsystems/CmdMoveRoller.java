package org.team3128.compbot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.team3128.common.generics.RobotConstants;

import org.team3128.common.NarwhalRobot;
import org.team3128.common.control.trajectory.Trajectory;
import org.team3128.common.control.trajectory.TrajectoryGenerator;
import org.team3128.common.control.trajectory.constraint.TrajectoryConstraint;
import org.team3128.common.drive.DriveCommandRunning;
import org.team3128.common.hardware.limelight.Limelight;
import org.team3128.common.hardware.gyroscope.Gyro;
import org.team3128.common.hardware.gyroscope.NavX;
import org.team3128.common.utility.units.Angle;
import org.team3128.common.utility.units.Length;
import org.team3128.common.vision.CmdHorizontalOffsetFeedbackDrive;
import org.team3128.compbot.subsystems.Constants;
import org.team3128.common.utility.Log;
import org.team3128.common.utility.RobotMath;
import org.team3128.common.utility.datatypes.PIDConstants;
import org.team3128.common.narwhaldashboard.NarwhalDashboard;
import org.team3128.common.listener.ListenerManager;
import org.team3128.common.listener.controllers.ControllerExtreme3D;
import org.team3128.common.listener.controltypes.Button;
import org.team3128.common.hardware.motor.LazyCANSparkMax;
import org.team3128.common.utility.math.Pose2D;
import org.team3128.common.utility.math.Rotation2D;
import org.team3128.common.utility.test_suite.CanDevices;
import org.team3128.common.utility.test_suite.ErrorCatcherUtility;
import org.team3128.compbot.subsystems.FalconDrive;
import org.team3128.compbot.subsystems.RobotTracker;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.team3128.compbot.main.MainCompbot;
import org.team3128.compbot.subsystems.*;

//import java.util.ArrayList;
import java.util.concurrent.*;

import org.team3128.common.generics.ThreadScheduler;
import org.team3128.common.generics.Threaded;
import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import java.util.Queue; 
import java.util.*;


public class CmdMoveRoller extends Command {
    
    private LazyCANSparkMax intake;
    private LazyCANSparkMax middle;
    private LazyCANSparkMax feeder;
    private LinkedList<Ball> ballList;

    public CmdMoveRoller(LazyCANSparkMax intake, LazyCANSparkMax middle, LazyCANSparkMax feeder, LinkedList<Ball> ballList){
        this.intake = intake;
        this.middle = middle;
        this.feeder = feeder;
        this.ballList = ballList;
        // should take in separate target positions for each roller
        // depending on position/number of balls
        // specifically middle roller should either have a low,
        // middle, or high target position
    }

    @Override
    protected void initialize() {
        //initialize :)
    }

    @Override
    protected void execute() {
        //set all motors to a power or maybe pid on velocity?, if motors
        //  should be moving
        //for each roller, depending on the number of balls inside and if
        //  intaking or shooting: if ball x is in position y, stop moving
        //for all sensors, if blocked, set inplace to true
        //  save current encoder position
        //if inplace is true and sensor becomes unblocked:
        //  set inplace to false
        //  average old and current encoder position
        //  ball just after this sensor's position gets encoder position reset
    }
    
    @Override
    protected boolean isFinished() {
        // if all motors are stopped return true
        // else return false and if we are shooting, move the middle motor
        // to the proper position and update ball positions
        return true;
    }
}
