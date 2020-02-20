package org.team3128.compbot.autonomous;

import com.esotericsoftware.minlog.Log;

import org.team3128.compbot.subsystems.Arm;
import org.team3128.compbot.subsystems.Arm.ArmState;

import edu.wpi.first.wpilibj.command.Command;

public class CmdSetArm extends Command {
    Arm arm;
    ArmState state;

    public CmdSetArm(Arm arm, ArmState state) {
        this.arm = arm;
        this.state = state; 
    }

    @Override
    protected void initialize() {
        arm.setState(state);
    }

    protected void execute() {
        /*
        probably not needed
        if (arm.ARM_STATE != state) {
            arm.setState(state);
        }
        */
        if (Math.abs(state.armAngle - arm.getAngle()) > 2) {
            Log.info("CmdSetArm", "Arm angle is off from set arm angle state, fixing");
            arm.update();
        }
    }

    protected boolean isFinished() {
        if (state.armAngle - arm.getAngle() < 2) {
            return true;
        } 
        else {
            return false;
        }
    }

}