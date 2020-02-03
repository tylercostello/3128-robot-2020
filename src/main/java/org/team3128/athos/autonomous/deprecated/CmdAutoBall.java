package org.team3128.athos.autonomous.deprecated;

import org.team3128.common.drive.DriveCommandRunning;
import org.team3128.common.hardware.gyroscope.Gyro;
import org.team3128.common.hardware.limelight.Limelight;
import org.team3128.common.utility.datatypes.PIDConstants;
import org.team3128.common.utility.units.Length;
import org.team3128.common.vision.CmdHorizontalOffsetFeedbackDrive;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class CmdAutoBall extends CommandGroup {
    public CmdAutoBall(Limelight ballLimelight, Gyro gyro, PIDConstants visionPID, PIDConstants blindPID, DriveCommandRunning cmdRunning) {
        addSequential(new CmdHorizontalOffsetFeedbackDrive(gyro, ballLimelight, 
        ballLimelight, cmdRunning, 3.5 *Length.in, visionPID, 0, 0, 0, blindPID, 
        ballLimelight.cameraAngle));
        /*
        CmdHorizontalOffsetFeedbackDrive moveToBall = 
        new CmdHorizontalOffsetFeedbackDrive(gyro, ballLimelight, 
        ballLimelight, null, 3.5 *Length.in, visionPID, 0, 0, 0, blindPID, 
        ballLimelight.cameraAngle);
        */
    }
}