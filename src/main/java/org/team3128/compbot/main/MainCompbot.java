package org.team3128.compbot.main;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

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
import org.team3128.common.utility.Log;
import org.team3128.common.utility.RobotMath;
import org.team3128.common.utility.datatypes.PIDConstants;
import org.team3128.common.narwhaldashboard.NarwhalDashboard;
import org.team3128.common.listener.ListenerManager;
import org.team3128.common.listener.POVValue;
import org.team3128.common.listener.controllers.ControllerExtreme3D;
import org.team3128.common.listener.controltypes.Button;
import org.team3128.common.listener.controltypes.POV;
import org.team3128.common.hardware.motor.LazyCANSparkMax;
import org.team3128.common.utility.math.Pose2D;
import org.team3128.common.utility.math.Rotation2D;
import org.team3128.common.utility.test_suite.CanDevices;
import org.team3128.common.utility.test_suite.ErrorCatcherUtility;
import org.team3128.compbot.commands.CmdAlignShoot;
import org.team3128.compbot.subsystems.*;
import org.team3128.compbot.subsystems.Constants;
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

import java.util.ArrayList;
import java.util.concurrent.*;

import org.team3128.common.generics.ThreadScheduler;

public class MainCompbot extends NarwhalRobot {

    public Command triggerCommand;
    private DriveCommandRunning driveCmdRunning;

    FalconDrive drive = FalconDrive.getInstance();
    Hopper hopper = Hopper.getInstance();
    Arm arm = Arm.getInstance();
    Shooter shooter = Shooter.getInstance();
    Intake intake = Intake.getInstance();
    RobotTracker robotTracker = RobotTracker.getInstance();

    ExecutorService executor = Executors.newFixedThreadPool(4);
    ThreadScheduler scheduler = new ThreadScheduler();
    Thread auto;

    public Joystick joystick;
    public ListenerManager lm;
    public Gyro gyro;
    public PowerDistributionPanel pdp;

    public NetworkTable table;
    public NetworkTable limelightTable;

    public double startTime = 0;

    public String trackerCSV = "Time, X, Y, Theta, Xdes, Ydes";

    public ArrayList<Pose2D> waypoints = new ArrayList<Pose2D>();
    public Trajectory trajectory;

    Limelight limelight = new Limelight("limelight-c", 26.0, 0, 0, 30);
    Limelight[] limelights = new Limelight[1];
    public ErrorCatcherUtility errorCatcher;
    public static CanDevices[] CanChain = new CanDevices[42];

    public static void setCanChain() {
        CanChain[0] = Constants.TestSuiteConstants.rightDriveLeader;
        CanChain[1] = Constants.TestSuiteConstants.rightDriveFollower;
        CanChain[2] = Constants.TestSuiteConstants.leftDriveFollower;
        CanChain[3] = Constants.TestSuiteConstants.leftDriveLeader;
        CanChain[4] = Constants.TestSuiteConstants.PDP;
    }

    @Override
    protected void constructHardware() {
        scheduler.schedule(drive, executor);
        scheduler.schedule(hopper, executor);
        scheduler.schedule(shooter, executor);
        scheduler.schedule(arm, executor);
        scheduler.schedule(intake, executor);
        scheduler.schedule(robotTracker, executor);

        driveCmdRunning = new DriveCommandRunning();

        // // Instatiator if we're using the NavX
        // gyro = new NavX();

        // // Instatiator if we're using the KoP Gyro
        // gyro = new AnalogDevicesGyro();
        // //gyro.recalibrate();

        joystick = new Joystick(1);
        lm = new ListenerManager(joystick);
        addListenerManager(lm);

        // initialization of auto test suite

        limelights[0] = limelight;
        pdp = new PowerDistributionPanel(0);
        // Constants.TestSuiteConstants.rightDriveLeader = new
        // CanDevices(CanDevices.DeviceType.FALCON, 0, "Right Drive Leader", null, null,
        // null, drive.rightTalon, null);
        // Constants.rightDriveFollower = new CanDevices(CanDevices.DeviceType.FALCON,
        // 1, "Right Drive Follower", null,
        // null, null, drive.rightTalonSlave, null);
        // Constants.leftDriveLeader = new CanDevices(CanDevices.DeviceType.FALCON, 2,
        // "Left Drive Leader", null, null,
        // null, drive.leftTalon, null);
        // Constants.leftDriveFollower = new CanDevices(CanDevices.DeviceType.FALCON, 3,
        // "Left Drive Follower", null, null,
        // null, drive.leftTalonSlave, null);
        // Constants.PDP = new CanDevices(CanDevices.DeviceType.PDP, 0, "Power
        // Distribution Panel", null, null, null, null,
        // pdp);
        setCanChain();
        errorCatcher = new ErrorCatcherUtility(CanChain, limelights, drive);

        NarwhalDashboard.addButton("ErrorCatcher", (boolean down) -> {
            if (down) {
                // Janky fix

                errorCatcher.testEverything();

                errorCatcher.testEverything();
            }
        });
    }

    @Override
    protected void constructAutoPrograms() {
    }

    @Override
    protected void setupListeners() {
        lm.nameControl(ControllerExtreme3D.TWIST, "MoveTurn");
        lm.nameControl(ControllerExtreme3D.JOYY, "MoveForwards");
        lm.nameControl(ControllerExtreme3D.THROTTLE, "Throttle");
        lm.nameControl(new Button(5), "ResetGyro");
        lm.nameControl(new Button(6), "PrintCSV");
        lm.nameControl(new Button(3), "ClearTracker");
        lm.nameControl(new Button(4), "ClearCSV");

        lm.addMultiListener(() -> {
            if (!driveCmdRunning.isRunning) {
                double horiz = -0.7 * lm.getAxis("MoveTurn");
                double vert = -1.0 * lm.getAxis("MoveForwards");
                double throttle = -1.0 * lm.getAxis("Throttle");

                drive.arcadeDrive(horiz, vert, throttle, true);
            }
        }, "MoveTurn", "MoveForwards", "Throttle");

        lm.nameControl(ControllerExtreme3D.TRIGGER, "AlignShoot");
        lm.addButtonDownListener("AlignShoot", () -> {
            triggerCommand = new CmdAlignShoot(drive, shooter, arm, hopper, gyro, limelight, driveCmdRunning,
                    Constants.VisionConstants.TX_OFFSET, 5);
            triggerCommand.start();
            Log.info("MainCompbot.java", "[Vision Alignment] Started");
        });
        lm.addButtonUpListener("AlignShoot", () -> {
            triggerCommand.cancel();
            triggerCommand = null;
            Log.info("MainCompbot.java", "[Vision Alignment] Stopped");
        });

        lm.addButtonDownListener("ResetGyro", () -> {
            drive.resetGyro();
        });
        lm.addButtonDownListener("PrintCSV", () -> {
            Log.info("MainCompbot", trackerCSV);
        });
        lm.addButtonDownListener("ClearCSV", () -> {
            trackerCSV = "Time, X, Y, Theta, Xdes, Ydes";
            Log.info("MainCompbot", "CSV CLEARED");
            startTime = Timer.getFPGATimestamp();
        });

        lm.addButtonDownListener("ClearTracker", () -> {
            robotTracker.resetOdometry();
        });

        lm.nameControl(new POV(0), "IntakePOV");
        lm.addListener("IntakePOV", (POVValue pov) -> {
            switch (pov.getDirectionValue()) {
            case 8:
            case 1:
            case 7:
                // push all balls backwards to clear hopper

                break;
            case 3:
            case 4:
            case 5:
                // start intake command

                // povCommand = new CmdAlignShoot(drive, shooter, arm, hopper, gyro, limelight,
                // driveCmdRunning,
                // povCommand.start();

                break;
            case 0:
                povCommand.cancel();
                povCommand = null;

                break;
            default:
                break;
            }
        });

    }

    @Override
    protected void teleopPeriodic() {

    }

    double maxLeftSpeed = 0;
    double maxRightSpeed = 0;
    double maxSpeed = 0;
    double minLeftSpeed = 0;
    double minRightSpeed = 0;
    double minSpeed = 0;

    double currentLeftSpeed;
    double currentLeftDistance;
    double currentRightSpeed;
    double currentRightDistance;
    double currentSpeed;
    double currentDistance;

    @Override
    protected void updateDashboard() {
        currentLeftSpeed = drive.getLeftSpeed();
        currentLeftDistance = drive.getLeftDistance();
        currentRightSpeed = drive.getRightSpeed();
        currentRightDistance = drive.getRightDistance();

        currentSpeed = drive.getSpeed();
        currentDistance = drive.getDistance();

        NarwhalDashboard.put("time", DriverStation.getInstance().getMatchTime());
        NarwhalDashboard.put("voltage", RobotController.getBatteryVoltage());

        SmartDashboard.putNumber("Gyro Angle", drive.getAngle());
        SmartDashboard.putNumber("Left Distance", currentLeftDistance);
        SmartDashboard.putNumber("Right Distance", currentRightDistance);

        SmartDashboard.putNumber("Distance", currentDistance);

        SmartDashboard.putNumber("Left Velocity", currentLeftSpeed);
        SmartDashboard.putNumber("Right Velocity", currentRightSpeed);

        SmartDashboard.putNumber("Velocity", drive.getSpeed());

        SmartDashboard.putNumber("RobotTracker - x:", robotTracker.getOdometry().getTranslation().getX());
        SmartDashboard.putNumber("RobotTracker - y:", robotTracker.getOdometry().getTranslation().getY());
        SmartDashboard.putNumber("RobotTracker - theta:", robotTracker.getOdometry().getRotation().getDegrees());

        maxLeftSpeed = Math.max(maxLeftSpeed, currentLeftSpeed);
        maxRightSpeed = Math.max(maxRightSpeed, currentRightSpeed);
        maxSpeed = Math.max(maxSpeed, currentSpeed);
        minLeftSpeed = Math.min(minLeftSpeed, currentLeftSpeed);
        minRightSpeed = Math.min(minRightSpeed, currentLeftSpeed);
        minSpeed = Math.min(minSpeed, currentSpeed);

        SmartDashboard.putNumber("Max Left Speed", maxLeftSpeed);
        SmartDashboard.putNumber("Min Left Speed", minLeftSpeed);
        SmartDashboard.putNumber("Max Right Speed", maxRightSpeed);
        SmartDashboard.putNumber("Min Right Speed", minRightSpeed);

        SmartDashboard.putNumber("Max Speed", maxSpeed);
        SmartDashboard.putNumber("Min Speed", minSpeed);

        trackerCSV += "\n" + String.valueOf(Timer.getFPGATimestamp() - startTime) + ","
                + String.valueOf(robotTracker.getOdometry().translationMat.getX()) + ","
                + String.valueOf(robotTracker.getOdometry().translationMat.getY()) + ","
                + String.valueOf(robotTracker.getOdometry().rotationMat.getDegrees()) + ","
                + String.valueOf(robotTracker.trajOdometry.translationMat.getX()) + ","
                + String.valueOf(robotTracker.trajOdometry.translationMat.getY());
    }

    @Override
    protected void teleopInit() {
        scheduler.resume();
    }

    @Override
    protected void autonomousInit() {
        waypoints.add(new Pose2D(0, 0, Rotation2D.fromDegrees(0)));
        waypoints.add(new Pose2D(0 * Constants.MechanismConstants.inchesToMeters,
                70 * Constants.MechanismConstants.inchesToMeters, Rotation2D.fromDegrees(-45)));

        trajectory = TrajectoryGenerator.generateTrajectory(waypoints, new ArrayList<TrajectoryConstraint>(), 0, 0,
                120 * Constants.MechanismConstants.inchesToMeters, 0.5, false);

        trackerCSV = "Time, X, Y, Theta, Xdes, Ydes";
        Log.info("MainCompbot", "going into autonomousinit");
        scheduler.resume();
        robotTracker.resetOdometry();
        drive.setAutoTrajectory(trajectory, false);
        drive.startTrajectory();
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    protected void disabledInit() {
        scheduler.pause();
    }

    public static void main(String... args) {
        RobotBase.startRobot(MainCompbot::new);
    }

    @Override
    public void endCompetition() {
        // TODO Auto-generated method stub

    }
}