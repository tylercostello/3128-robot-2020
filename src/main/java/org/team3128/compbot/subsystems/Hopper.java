package org.team3128.compbot.subsystems;

import org.team3128.common.utility.Log;

import edu.wpi.first.wpilibj.DigitalInput;

import org.team3128.common.generics.Threaded;
import org.team3128.common.hardware.motor.LazyCANSparkMax;

import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.LinkedList;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.team3128.common.game_elements.Ball;

public class Hopper extends Threaded {

    public enum HopperState {
        POS_0(new boolean[] { false, false, false, false }),
        POS_1(new boolean[] { false, false, false, true }),
        POS_2(new boolean[] { false, false, true, true }),
        POS_3(new boolean[] { false, true, true, true }),
        POS_4(new boolean[] { true, true, true, true });
        //POS_5(new boolean[] { true, true, true, true }),
        //POS_6(new boolean[] { false, true, true, true });

        public boolean[] hopperState;

        private HopperState(boolean[] hopperState) {
            this.hopperState = hopperState;
        }
    }

    public enum ActionState {
        STANDBY,
        INTAKING,
        SHOOTING,
        ORGANIZING;

        private ActionState() {

        }
    }

    public LazyCANSparkMax INTAKE_MOTOR, HOPPER_FEEDER_MOTOR, CORNER_MOTOR, GATEKEEPER_MOTOR;
    public CANEncoder HOPPER_FEEDER_ENCODER, CORNER_ENCODER;
    public DigitalInput SENSOR_0, SENSOR_1;

    boolean[] ballArray = { false, false, false, false };
    private static final Hopper instance = new Hopper();

    //private boolean isMoving;
    private boolean isShooting, isIntaking;
    private boolean empty0 = true;
    private boolean empty1 = true;
    private boolean isFeeding = false;
    private boolean isLoading = false;
    //private boolean isOrganizing = false;
    private boolean openTheGates = false;
    private double startPos = 0;
    private int ballCount;

    public ActionState actionState;

    private DigitalInput[] sensorPositions = { SENSOR_0, SENSOR_1 }; // top to bottom //0 = 1.5, 1 = 3, 2 = 4
                                                                               // mathhh

    public static Hopper getInstance() {
        return instance;
    }

    private Hopper() {
        configMotors();
        configEncoders();
        configSensors();
        ballCount = 0; //TODO: initial ball count
    }

    @Override
    public void update() {

        switch (actionState) {
            case STANDBY:
                standbyIntake();

            case INTAKING:
                standbyIntake();

            case SHOOTING:
                loadShoot();
            
            case ORGANIZING:
                organize();
        }
    }

    private void configMotors() {
        INTAKE_MOTOR = new LazyCANSparkMax(Constants.IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
        HOPPER_FEEDER_MOTOR = new LazyCANSparkMax(Constants.HopperConstants.HOPPER_FEEDER_MOTOR_ID,
                MotorType.kBrushless);
        CORNER_MOTOR = new LazyCANSparkMax(Constants.HopperConstants.CORNER_MOTOR_ID, MotorType.kBrushless);
        GATEKEEPER_MOTOR = new LazyCANSparkMax(Constants.HopperConstants.GATEKEEPER_MOTOR_ID, MotorType.kBrushless);
    }

    private void configEncoders() {
        HOPPER_FEEDER_ENCODER = HOPPER_FEEDER_MOTOR.getEncoder();
        CORNER_ENCODER = CORNER_MOTOR.getEncoder();
    }

    private void configSensors() {
        SENSOR_0 = new DigitalInput(Constants.HopperConstants.SENSOR_0_ID);
        SENSOR_1 = new DigitalInput(Constants.HopperConstants.SENSOR_1_ID);
    }

    public void setBallOrder(boolean[] CurrBallArray) {
        if (CurrBallArray.length == ballArray.length) {
            for (int i = 0; i < ballArray.length; i++) {
                ballArray[i] = CurrBallArray[i];
            }
        }
    }

    public boolean isReady() {
        return false; // TODO: only return true if there is a ball ready to be shot AND the shoot
                      // method isn't currently in use.
    }

    public int getNumBalls() {
        int ball_count = 0;
        for (boolean ball : ballArray) {
            if (ball) {
                ball_count++;
            }
        }
        return ball_count;
    }

    public boolean isEmpty() {
        return (getNumBalls() == 0);
    }

    public boolean isFull() {
        return (getNumBalls() == Constants.HopperConstants.CAPACITY - 1);
    }

    public boolean[] shift(boolean[] in_array) {
        boolean[] out_array = new boolean[in_array.length];
        for (int i = 0; i < in_array.length - 1; i++) {
            out_array[i] = in_array[i + 1];
        }
        out_array[in_array.length - 1] = false;
        return out_array;
    }

    private boolean[] addBall(boolean[] in_array) {
        boolean[] out_array = new boolean[in_array.length];
        // for (int i = 0; i < in_array.length; i++) {
        for (int i = in_array.length - 1; i >= 0; i--) {
            boolean added = false;
            if (in_array[i]) {
                out_array[i] = in_array[i];
            } else if (!added) {
                out_array[i] = true;
                added = true;
            }
        }
        return out_array;
    }

    public boolean[] getBallArray() {
        return ballArray;
    }

    public void updateBallArray(boolean[] in_array) {
        this.ballArray = in_array;
    }

    // public void setIsMoving(boolean isMoving) {
    //     this.isMoving = isMoving;
    // }

    public int getFirstAvailablePos() {
        int tempCount = 0;
        int record_holder_index = 0;
        for (boolean ball : ballArray) {
            if (ball) {
                record_holder_index = tempCount;
            }
            tempCount++;
        }
        return record_holder_index + 1;
    }

    public void setMotorPowers(double p_Gatekeeer, double p_Corner, double p_HopperFeeder) {
        GATEKEEPER_MOTOR.set(p_Gatekeeer);
        CORNER_MOTOR.set(p_Corner);
        HOPPER_FEEDER_MOTOR.set(p_HopperFeeder);
    }

    public void gateKeep(boolean value) {
        openTheGates = value;
    }

    // public void setIsOrganizing(boolean value) {
    //     isOrganizing = value;
    // }

    public void shoot() {
        gateKeep(true);
    }

    public void setAction(ActionState state) {
        this.actionState = state;
        if (state == ActionState.INTAKING) {
            INTAKE_MOTOR.set(Constants.IntakeConstants.INTAKE_MOTOR_ON_VALUE);
        } else {
            INTAKE_MOTOR.set(Constants.IntakeConstants.INTAKE_MOTOR_OFF_VALUE);
        }
    }

    public void standbyIntake() {
        if (!isFull()) {
            setMotorPowers(0, 0, Constants.HopperConstants.BASE_POWER);
        } else {
            setMotorPowers(0, 0, 0);
        }
        if (SENSOR_1.get()) {
            empty1 = false;
            if (!isFeeding) {
                setMotorPowers(0, Constants.HopperConstants.BASE_POWER, Constants.HopperConstants.BASE_POWER);
            } else {
                setMotorPowers(0, Constants.HopperConstants.BALL_SPACING, 0);
            }
        } else if (!empty1) {
            empty1 = true;
            ballCount++;

            updateBallArray(addBall(getBallArray()));

            startPos = CORNER_ENCODER.getPosition();
            setMotorPowers(0, Constants.HopperConstants.BASE_POWER, 0);
            isFeeding = true;
        }
        if (isFeeding) {
            setMotorPowers(0, Constants.HopperConstants.BASE_POWER, 0);
            if (CORNER_ENCODER.getPosition() - Constants.HopperConstants.BALL_SPACING >= startPos) {
                isFeeding = false;
            }
        }
    }

    public void loadShoot() {
        if (!SENSOR_0.get()) {
            if (!empty0 && openTheGates) {
                ballCount--;
                updateBallArray(shift(getBallArray()));
                openTheGates = false;
            }
            empty0 = true;
            setMotorPowers(0, Constants.HopperConstants.BASE_POWER, Constants.HopperConstants.BASE_POWER);
        } else if (empty0) {
            empty0 = false;

            startPos = CORNER_ENCODER.getPosition();
            setMotorPowers(0, Constants.HopperConstants.BASE_POWER, Constants.HopperConstants.BASE_POWER);
            isLoading = true;
        } else if (openTheGates && !isLoading) {
            setMotorPowers(Constants.HopperConstants.GATEKEEPER_POWER, Constants.HopperConstants.BASE_POWER, Constants.HopperConstants.BASE_POWER);
        }
        if (isLoading) {
            setMotorPowers(0, Constants.HopperConstants.BASE_POWER, Constants.HopperConstants.BASE_POWER);
            if (CORNER_ENCODER.getPosition() - Constants.HopperConstants.SHOOTER_SPACING >= startPos) {
                isLoading = false;
            }
        }
        if (SENSOR_1.get()) {
            empty1 = false;
        } else if (!empty1) {
            empty1 = true;
            ballCount++;
            updateBallArray(addBall(getBallArray()));
        }
    }

    public void organize() {
        if (isEmpty()) {
            setAction(ActionState.STANDBY);
        }
        if (!SENSOR_1.get()) {
            empty1 = true;
            setMotorPowers(0, -Constants.HopperConstants.BASE_POWER, Constants.HopperConstants.BASE_POWER);
        } else if (empty1) {
            empty1 = false;
            setAction(ActionState.STANDBY);
        }
    }
}