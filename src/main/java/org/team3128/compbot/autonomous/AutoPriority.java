package org.team3128.compbot.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;

import org.team3128.common.utility.math.Pose2D;
import org.team3128.common.utility.math.Rotation2D;
import org.team3128.common.drive.Drive;
import org.team3128.common.hardware.gyroscope.Gyro;
import org.team3128.common.hardware.limelight.Limelight;
import org.team3128.common.drive.DriveCommandRunning;
import org.team3128.compbot.autonomous.*;
import org.team3128.compbot.commands.*;
import org.team3128.compbot.subsystems.Constants.VisionConstants;
import org.team3128.compbot.subsystems.*;

public class AutoPriority extends CommandGroup {


    public AutoPriority(FalconDrive drive, Shooter shooter, Arm arm, Hopper hopper, Gyro gyro, Limelight limelight, DriveCommandRunning cmdRunning, double timeoutMs) {       
        addSequential(new CmdAlignShoot(drive, shooter, arm, hopper, gyro, limelight, cmdRunning, Constants.VisionConstants.TX_OFFSET, 3));
        addSequential(new CmdAutoTrajectory(drive, 120, 0.5, 10000, 
            new Pose2D(0, 0, Rotation2D.fromDegrees(180))));
    }
}