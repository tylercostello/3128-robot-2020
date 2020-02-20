package org.team3128.compbot.autonomous;

import org.team3128.compbot.subsystems.Arm;
import org.team3128.compbot.subsystems.Arm.ArmState;

import edu.wpi.first.wpilibj.command.Command;

public class CmdArmInitialize extends Command {
    Arm arm;
    Command setArmCommand;

    public CmdArmInitialize(Arm arm) {
        this.arm = arm; 
    }

    @Override
    protected void initialize() {
        setArmCommand = new CmdSetArm(arm, ArmState.STARTING_LOW);
        setArmCommand.start();
    }

    protected void execute() {
        if (arm.isReady && Math.abs(ArmState.STARTING_LOW.armAngle - arm.getAngle()) < 5) { //TODO: make arm.isready
            setArmCommand = new CmdSetArm(arm, ArmState.STARTING);
            setArmCommand.start();
        }
    }

    protected boolean isFinished() {
        if (arm.isReady && Math.abs(ArmState.STARTING.armAngle - arm.getAngle()) < 3) {
            return true;
        }
        else {
            return false;
        }
    }

}