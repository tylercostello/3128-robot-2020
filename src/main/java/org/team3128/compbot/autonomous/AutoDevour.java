// Authors: Mika, Thomas, Mason, Sohan
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
import com.kauailabs.navx.frc.AHRS;

public class AutoDevour extends CommandGroup {

    public AutoDevour(FalconDrive drive, Shooter shooter, Arm arm, Hopper hopper, AHRS ahrs, Limelight limelight, DriveCommandRunning cmdRunning, double timeoutMs) {       
        addSequential(new CmdAlignShoot(drive, shooter, arm, hopper, ahrs, limelight, cmdRunning, Constants.VisionConstants.TX_OFFSET, 3));
        for (int i = 0; i < 3; i++) { // picking up three balls
            addSequential(new CmdIntake(hopper, arm));
        }
        addSequential(new CmdAlignShoot(drive, shooter, arm, hopper, ahrs, limelight, cmdRunning, Constants.VisionConstants.TX_OFFSET, 3));
        
        addSequential(new CmdAutoTrajectory(drive, 120, 0.5, 10000, 
            new Pose2D(0, 0, Rotation2D.fromDegrees(0)),
            new Pose2D(-12 * Constants.MechanismConstants.inchesToMeters, -12 * Constants.MechanismConstants.inchesToMeters, Rotation2D.fromDegrees(0)), 
            new Pose2D(12 * Constants.MechanismConstants.inchesToMeters, 12 * Constants.MechanismConstants.inchesToMeters, Rotation2D.fromDegrees(0))));

        // if enough time run AutoRendezvous
    }
}