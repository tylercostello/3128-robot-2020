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


public class Hopper extends Threaded {
    public int countBalls = 0;
    public DigitalInput digitalInput;
    public DigitalInput digitalInput2;
    public static LazyCANSparkMax intake;
    public static LazyCANSparkMax middle;
    public static LazyCANSparkMax feeder;
    public boolean inPlace = false;
    public boolean inPlace2 = false;
    public LinkedList<Ball> ballList = new LinkedList<Ball>();
    public Hopper (){
        
    }

    public void hoppityHop(LazyCANSparkMax intake, LazyCANSparkMax middle, LazyCANSparkMax feeder) {
        
        digitalInput = new DigitalInput(0);
        digitalInput2 = new DigitalInput(1);

        //intake = new LazyCANSparkMax(0, MotorType.kBrushless);
        CANEncoder intakeEncoder = intake.getEncoder();

        //middle = new LazyCANSparkMax(0, MotorType.kBrushless);
        CANEncoder middleEncoder = middle.getEncoder();

        //feeder = new LazyCANSparkMax(0, MotorType.kBrushless);
        CANEncoder feederEncoder = feeder.getEncoder();

    
        // logic for photoelectric sensor 
        if (inPlace == false && digitalInput.get()){
            //countBalls++;
            inPlace = true;
            ballList.add(new Ball());
            System.out.println("Number of balls: " + ballList.size());
            
            switch(ballList.size()) {
                case 1:
                    //intake.moveOneBall();
                    //middle.movethree or whatever
                    //feeder.half ball
                    //for ball in balllist update position
                    break;
                case 2:
                    //intake.moveone
                    //middle.move half
                    //ballArray[1] = true;
                    break;
                case 3:
                    //intake one
                    //middle move hakdsggasdf
                    //ballArray[2] = true;
                    break;
                case 4:
                    //intake one
                    //middle move hasdkfadsgasdff
                    //ballArray[3] = true;
                    break;
                case 5:
                    //intake halff
                    //ballArray[0] = true;
                    break;
                default:
                    Log.info("hoppity hop hop", "yuckity yuck yuck something brokity broke broke");
                    Log.info("hoppity hop hop", String.valueOf(1 / 0));
                    break;
            }
        } else if (!digitalInput.get()) {
            inPlace = false;
        }
        
        if (inPlace2 == false && digitalInput2.get()){
            //countBalls--;
            //System.out.println("Number of balls: " + countBalls);
            ballList.removeFirst();
            inPlace2 = true;

            switch(ballList.size()) {
                case 1:
                    //everything.move half
                    //shoot
                    break;
                case 2:
                    //feeder.move half
                    //shoot
                    //middle.move half
                    //intake move half
                    break;
                case 3:
                    //everything move half
                    //shoot
                    break;
                case 4:
                    //intake one
                    //middle move hasdkfadsgasdff
                    //ballArray[3] = true;
                    break;
                case 5:
                    //intake halff
                    //ballArray[0] = true;
                    break;
                default:
                    Log.info("hoppity hop hop", "yuck ity yuck yuck something brokity broke broke");
                    Log.info("hoppity hop hop", String.valueOf(1 / 0));
                    break;
            }
        }
        else if (!digitalInput2.get()) {
            inPlace2 = false;
        }
    }

    public void moveRollers() {
        //takes in roller motors and ball objects, moves the roller motors 
        // and updates the ball object locations using encoder values
        //somehow needs to account for ball location when moving between 
        // rollers and also has to reset the encoder values when photoelectric
        // sensor is triggered.
        // like when sensor a is triggered and then untriggered, set ball a's 
        // position to half the distance the rollers moved from trigger to
        // untrigger

        //also we probably want this as a separate class so it can be a command
    }

    @Override
    public void update() {
        // TODO Auto-generated method stub

    }
}