/** @author:Jude, Daniel, Tyler, Jenny, Priyanka */

package org.team3128.compbot.commands;

import java.lang.invoke.WrongMethodTypeException;
import java.security.GuardedObject;
import java.util.Arrays;

import org.team3128.compbot.subsystems.Constants;
import org.team3128.compbot.subsystems.Hopper;
import org.team3128.compbot.subsystems.Intake;
import org.team3128.compbot.subsystems.Hopper.HopperState;

import edu.wpi.first.wpilibj.command.Command;

import org.team3128.common.utility.Log;

import edu.wpi.first.wpilibj.Timer;


public class CmdEjectBalls extends Command {
    Hopper hopper;
    Intake intake;
    int adjustBallCount;
    boolean[] updatedArray = {false, false, false, false};
    double currentTime;
    double startTime;
    int timeCounter = 0;
    

    public CmdEjectBalls(Hopper hopper, Intake intake) {
        this.hopper = hopper;
        this.intake = intake;
    }

    @Override
    protected void initialize() {
        //nothing here
        startTime = Timer.getFPGATimestamp();
        hopper.setMotorPowers(-Constants.HopperConstants.GATEKEEPER_POWER, -Constants.HopperConstants.BASE_POWER, -Constants.HopperConstants.BASE_POWER);
        intake.turnOnBackwards();
    }

    @Override
    protected void execute() {
        if (hopper.detectsBall(hopper.SENSOR_1)){
            adjustBallCount--;
        }
        timeCounter++;
        if (timeCounter == 25) {
            currentTime = Timer.getFPGATimestamp();
            timeCounter = 0;
        }
    }

    @Override
    protected boolean isFinished() {
        return (adjustBallCount == 0) || ((currentTime - startTime) >= 10000); //if the adjusted ball count is 0 or 10 seconds has gone by, return true
    }

    @Override
    protected void end() {
    }

    @Override
    protected void interrupted() {
        end();
    }
}