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
import org.team3128.athos.subsystems.Constants;
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

//import java.util.ArrayList;
import java.util.concurrent.*;

import org.team3128.common.generics.ThreadScheduler;
import org.team3128.common.generics.Threaded;


public class Hopper extends Threaded {
    public int countBalls = 0;
    public DigitalInput digitalInput;
    public DigitalInput digitalInput2;
    public static LazyCANSparkMax intake;
    public static LazyCANSparkMax middle;
    public static LazyCANSparkMax feeder;
    public boolean inPlace = false;
    public boolean inPlace2 = false;
    public Hopper (){
        
    }

    public void stopBreakingStuff(){
        digitalInput = new DigitalInput(0);
        digitalInput2 = new DigitalInput(1);
        intake = new LazyCANSparkMax(0, MotorType.kBrushless);
        middle = new LazyCANSparkMax(0, MotorType.kBrushless);
        feeder = new LazyCANSparkMax(0, MotorType.kBrushless);
            // logic for photoelectric sensor 
            if (inPlace == false && digitalInput.get()){
                countBalls++;
            // System.out.println("Number of balls: " + countBalls);
                inPlace = true;

                if (countBalls == 1){
                    //intake on
                    //middle on

                }
                if (countBalls == 2){
                    //short intake
                    //middle off

                }
                if (countBalls == 3){
                    //short intake
                    //middle off
                }
                if (countBalls == 4){
                    //intake on until 2 balls pass corner sensor
                    //middle on until 1 ball passes top hopper sensor
                    //intake low
                }
                if (countBalls == 5){
                    //intake low
                }


            }
            else if (!digitalInput.get()) {
                inPlace = false;
            }
            
            if (inPlace2 == false && digitalInput2.get()){
                countBalls--;
                //System.out.println("Number of balls: " + countBalls);
                inPlace2 = true;

                if (countBalls == 1){
                    //intake on
                    //middle on

                }
                if (countBalls == 2){
                    //intake short
                    //middle on until top hopper sensor
                }
                if (countBalls == 3){
                    //middle short
                    //middle reverse until ball on intake belt 
                }
                if (countBalls == 4){
                    //middle on
                    //short intake
                }


            }
            else if (!digitalInput2.get()) {
                inPlace2 = false;
            }
    }

    @Override
    public void update() {
        // TODO Auto-generated method stub

    }
}