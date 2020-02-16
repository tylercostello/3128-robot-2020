// authors:
//   The Cult of Mason

package org.team3128.compbot.commands;

import java.lang.invoke.WrongMethodTypeException;
import java.security.GuardedObject;
import java.util.Arrays;

import org.team3128.compbot.subsystems.Constants;
import org.team3128.compbot.subsystems.Hopper;
import org.team3128.compbot.subsystems.Hopper.HopperState;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.command.Command;

import org.team3128.common.utility.Log;

public class CmdIntake extends Command {
    Hopper hopper;
    int topCount;
    int middleCount;
    int totalCount;
    boolean empty0 = false;
    boolean empty1 = false;
    boolean empty2 = false;
    boolean broken = false;
    double currentTime;
    double brokenTime;
    int position = 0;
    boolean[] updatedArray = new boolean[Constants.HopperConstants.CAPACITY];

    boolean inFocus = false;
    int focusPos = 0;

    public CmdIntake(Hopper hopper) {
        this.hopper = hopper;
    }

    @Override
    protected void initialize() {
        // nothing here
        switch(hopper.getNumBalls()) {
            case 0:
                topCount = 0;
                middleCount = 0;
                totalCount = 0;
                break;
            case 1:
                topCount = 1;
                middleCount = 0;
                totalCount = 1;
                break;
            case 2:
                topCount = 2;
                middleCount = 0;
                totalCount = 2;
                break;
            case 3:
                totalCount = 3;
                if (Arrays.equals(hopper.getBallArray(), Hopper.HopperState.POS_3.hopperState)) {
                    topCount = 2;
                    middleCount = 1;
                } else if (Arrays.equals(hopper.getBallArray(), Hopper.HopperState.POS_6.hopperState)){
                    topCount = 1;
                    middleCount = 2;
                } else {
                    Log.info("CmdIntake", "there are 3 balls in the shooter in a weird configuration");
                }
                break;
            case 4:
                topCount = 2;
                middleCount = 2;
                totalCount = 4;
                break;
            case 5:
                topCount = 2;
                middleCount = 2;
                totalCount = 5;
                break;
        }
        
        arm.zero();

    }

    @Override
    protected void execute() {
        updateCount();
        updatedArray = hopper.getBallArray();
        if (!hopper.isFull()) {
            hopper.INTAKE_MOTOR.set(Constants.IntakeConstants.INTAKE_MOTOR_ON_VALUE);
        } else {
            hopper.INTAKE_MOTOR.set(0);
        }
        switch(middleCount + topCount) {
            case 0:
                updatedArray = new boolean[] {false, false, false, false, false};
                if (totalCount > (middleCount + topCount)) {
                    hopper.setMotorPowers(0, Constants.HopperConstants.BASE_POWER, Constants.HopperConstants.BASE_POWER);
                } else {
                    hopper.setMotorPowers(0, 0, Constants.HopperConstants.BASE_POWER);
                }
                break;
            case 1:
                updatedArray = new boolean[] {true, false, false, false, false};
                if (totalCount > (middleCount + topCount)) {
                    hopper.setMotorPowers(0, Constants.HopperConstants.BASE_POWER, Constants.HopperConstants.BASE_POWER);
                    //boolean[] updatedArray = {true, false, false, false, true};
                } else if (topCount < (topCount + middleCount)) {
                    hopper.setMotorPowers(0, Constants.HopperConstants.BASE_POWER, Constants.HopperConstants.BASE_POWER);
                    //boolean[] updatedArray = {true, false, false, false, false};
                } else {
                    hopper.setMotorPowers(0, 0, Constants.HopperConstants.BASE_POWER);
                }
                break;
            case 2:
                updatedArray = new boolean[] {true, true, false, false, false};
                if (topCount < 2) {
                    hopper.setMotorPowers(0, Constants.HopperConstants.BASE_POWER, 0);
                } else if (totalCount > (middleCount + topCount)) {
                    hopper.setMotorPowers(0, Constants.HopperConstants.BASE_POWER, Constants.HopperConstants.BASE_POWER);
                } else {
                    hopper.setMotorPowers(0, 0, Constants.HopperConstants.BASE_POWER);
                }
                break;
            case 3:
                updatedArray = new boolean[] {true, true, false, true, false};
                if (topCount < 2) {
                    //Log.info("CmdIntake", "something screwed up; topCount should be 2 but it isn't");
                    Log.info("CmdIntake", "this had better have happened only because we just shot 2 balls and then reorganized");
                }
                if (totalCount > (middleCount + topCount)) {
                    hopper.setMotorPowers(0, Constants.HopperConstants.BASE_POWER, Constants.HopperConstants.BASE_POWER);
                } else {
                    hopper.setMotorPowers(0, 0, Constants.HopperConstants.BASE_POWER);
                }
                break;
            case 4:
                if (totalCount > (middleCount + topCount)) {
                    hopper.setMotorPowers(0, 0, 0);
                    updatedArray = new boolean[] {true, true, true, true, true};
                } else{
                    hopper.setMotorPowers(0, 0, Constants.HopperConstants.BASE_POWER);
                    updatedArray = new boolean[] {true, true, true, true, false};
                }
                break;
        }

        hopper.updateBallArray(updatedArray);
    }

    @Override
    protected boolean isFinished() {
        if (hopper.getNumBalls() == Constants.HopperConstants.CAPACITY) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    protected void end() {
        // handle what happens when the command is terminated
        brokenTime = Timer.getFPGATimestamp();
        currentTime = brokenTime;
        hopper.INTAKE_MOTOR.set(0);
        if (hopper.getNumBalls() == 5) {
            hopper.setMotorPowers(0, 0, 0);
        } else {
            hopper.setMotorPowers(0, 0, Constants.HopperConstants.BASE_POWER);
        }
        while(!broken) {
            currentTime = Timer.getFPGATimestamp();
            updateCount();
            switch(middleCount + topCount) {
                case 0:
                    updatedArray = new boolean[] {false, false, false, false, false};
                    if (totalCount > (middleCount + topCount)) {
                        hopper.setMotorPowers(0, Constants.HopperConstants.BASE_POWER, Constants.HopperConstants.BASE_POWER);
                    } else {
                        hopper.setMotorPowers(0, 0, Constants.HopperConstants.BASE_POWER);
                        if(currentTime - 250 > brokenTime) {
                            broken = true;
                        }
                    }
                    break;
                case 1:
                    updatedArray = new boolean[] {true, false, false, false, false};
                    if (totalCount > (middleCount + topCount)) {
                        hopper.setMotorPowers(0, Constants.HopperConstants.BASE_POWER, Constants.HopperConstants.BASE_POWER);
                        //boolean[] updatedArray = {true, false, false, false, true};
                    } else if (topCount < (topCount + middleCount)) {
                        hopper.setMotorPowers(0, Constants.HopperConstants.BASE_POWER, Constants.HopperConstants.BASE_POWER);
                        //boolean[] updatedArray = {true, false, false, false, false};
                    } else {
                        hopper.setMotorPowers(0, 0, Constants.HopperConstants.BASE_POWER);
                        if(currentTime - 250 > brokenTime) {
                            broken = true;
                        }
                    }
                    break;
                case 2:
                    updatedArray = new boolean[] {true, true, false, false, false};
                    if (topCount < 2) {
                        hopper.setMotorPowers(0, Constants.HopperConstants.BASE_POWER, 0);
                    } else if (totalCount > (middleCount + topCount)) {
                        hopper.setMotorPowers(0, Constants.HopperConstants.BASE_POWER, Constants.HopperConstants.BASE_POWER);
                    } else {
                        hopper.setMotorPowers(0, 0, Constants.HopperConstants.BASE_POWER);
                        if(currentTime - 250 > brokenTime) {
                            broken = true;
                        }
                    }
                    break;
                case 3:
                    updatedArray = new boolean[] {true, true, false, true, false};
                    if (topCount < 2) {
                        Log.info("CmdIntake", "something screwed up; topCount should be 2 but it isn't");
                    }
                    if (totalCount > (middleCount + topCount)) {
                        hopper.setMotorPowers(0, Constants.HopperConstants.BASE_POWER, Constants.HopperConstants.BASE_POWER);
                    } else {
                        hopper.setMotorPowers(0, 0, Constants.HopperConstants.BASE_POWER);
                        if(currentTime - 250 > brokenTime) {
                            broken = true;
                        }
                    }
                    break;
                case 4:
                    if (totalCount > (middleCount + topCount)) {
                        hopper.setMotorPowers(0, 0, 0);
                        updatedArray = new boolean[] {true, true, true, true, true};
                    } else{
                        hopper.setMotorPowers(0, 0, Constants.HopperConstants.BASE_POWER);
                        updatedArray = new boolean[] {true, true, true, true, false};
                    }
                    if(currentTime - 250 > brokenTime) {
                        broken = true;
                    }
                    break;
            }
        }
        hopper.setMotorPowers(0, 0, 0);
    }

    @Override
    protected void interrupted() {
        end(); // check if the balls are in a good position. If not, then shuffle them around
               // to the right position
    }

    private void updateCount() {
        if (!hopper.SENSOR_0.get()) {
            empty0 = true;
        } else if (empty0) {
            totalCount++;
            empty0 = false;
        }
        if (!hopper.SENSOR_1.get()) {
            empty1 = true;
        } else if (empty1) {
            middleCount++;
            totalCount++;
            empty1 = false;
        }
        if (!hopper.SENSOR_2.get()) {
            empty2 = true;
        } else if (empty2) {
            middleCount--;
            topCount++;
            totalCount++;
            empty2 = false;
        }
    }
}