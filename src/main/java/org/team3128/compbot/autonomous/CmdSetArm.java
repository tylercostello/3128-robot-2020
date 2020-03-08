package org.team3128.compbot.autonomous;

import org.team3128.compbot.subsystems.Constants;
import edu.wpi.first.wpilibj.command.Command;
import org.team3128.compbot.subsystems.Arm.ArmState;
import org.team3128.compbot.subsystems.Arm;
import org.team3128.common.utility.Log;
import edu.wpi.first.wpilibj.Timer;

public class CmdSetArm extends Command {
    
    Arm arm;
    ArmState armState;
    double timeoutMs, startTime;
    
    public CmdSetArm(Arm arm, ArmState armState, double timeoutMs) {
        this.arm = arm;
        this.armState = armState;
        this.timeoutMs = timeoutMs;
    }
    
    @Override
    protected void initialize() {
        arm.setState(armState);
        startTime = Timer.getFPGATimestamp();
    }
    
    @Override
    protected boolean isFinished() {
        if (arm.isReady() || ((Timer.getFPGATimestamp() - startTime) >= timeoutMs)){
            return true;
        } else {
            return false;
        }
    }

}