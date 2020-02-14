import java.lang.invoke.WrongMethodTypeException;
import java.security.GuardedObject;
import java.util.Arrays;

import org.team3128.compbot.subsystems.Constants;
import org.team3128.compbot.subsystems.Hopper;
import org.team3128.compbot.subsystems.Hopper.HopperState;

import edu.wpi.first.wpilibj.command.Command;

public class CmdIntake extends Command {
    Hopper hopper;
    int fillCount;

    boolean inFocus = false;
    int focusPos = 0;

    public CmdIntake(Hopper hopper) {
        this.hopper = hopper;
    }

    @Override
    protected void initialize() {
        // nothing here
        if (!hopper.isFull()) {
            hopper.INTAKE_MOTOR.set(Constants.IntakeConstants.INTAKE_MOTOR_ON_VALUE);
        }
    }

    @Override
    protected void execute() {
        if (!inFocus && hopper.SENSOR_4.get()) {
            inFocus = true;
            focusPos = hopper.getFirstAvailablePos(); // TODO: add gap support lol
        } else if (hopper.SENSOR_4.get()) {

        }
        if (inFocus) {
            switch (focusPos) {
            case 0:
                if (!hopper.SENSOR_0.get()) {
                    hopper.setIsMoving(true);
                    hopper.CORNER_MOTOR.set(Constants.HopperConstants.BASE_POWER);
                    hopper.SHOOTER_FEEDER_MOTOR.set(Constants.HopperConstants.BASE_POWER);
                } else {
                    inFocus = false;
                    hopper.setIsMoving(false);
                    hopper.CORNER_MOTOR.set(0);
                    hopper.SHOOTER_FEEDER_MOTOR.set(0);
                }
            case 1:
                if (!hopper.SENSOR_1.get()) {
                    hopper.setIsMoving(true);
                    hopper.CORNER_MOTOR.set(Constants.HopperConstants.BASE_POWER);
                    hopper.SHOOTER_FEEDER_MOTOR.set(Constants.HopperConstants.BASE_POWER);
                } else {
                    inFocus = false;
                    hopper.setIsMoving(false);
                    hopper.CORNER_MOTOR.set(0);
                    hopper.SHOOTER_FEEDER_MOTOR.set(0);
                }
            case 2:
                if (!hopper.SENSOR_2.get()) {
                    hopper.setIsMoving(true);
                    hopper.CORNER_MOTOR.set(Constants.HopperConstants.BASE_POWER);
                } else {
                    inFocus = false;
                    hopper.setIsMoving(false);
                    hopper.CORNER_MOTOR.set(0);
                }
            case 3:
                if (!hopper.SENSOR_3.get()) {
                    hopper.setIsMoving(true);
                    hopper.CORNER_MOTOR.set(Constants.HopperConstants.BASE_POWER);
                } else {
                    inFocus = false;
                    hopper.setIsMoving(false);
                    hopper.CORNER_MOTOR.set(0);
                }
            case 4:
                if (!hopper.SENSOR_4.get()) {
                    hopper.setIsMoving(true);
                    hopper.CORNER_MOTOR.set(Constants.HopperConstants.BASE_POWER);
                } else {
                    inFocus = false;
                    hopper.setIsMoving(false);
                    hopper.CORNER_MOTOR.set(0);
                }
            }
        }
    }

    @Override
    protected boolean isFinished() {
        if (hopper.getNumBalls() == Constants.HopperConstants.CAPACITY) {
            return true;
        }

        return false;
    }

    @Override
    protected void end() {
        // handle what happens when the command is terminated
        hopper.CORNER_MOTOR.set(0);
        hopper.SHOOTER_FEEDER_MOTOR.set(0);
        hopper.INTAKE_MOTOR.set(0);
    }

    @Override
    protected void interrupted() {
        end(); // check if the balls are in a good position. If not, then shuffle them around
               // to the right position
    }
}