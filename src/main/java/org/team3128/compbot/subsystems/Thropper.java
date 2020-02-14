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

public class Thropper extends Threaded {

    public enum ThropperState {
        POS_0(new boolean[] { false, false, false, false, false }),
        POS_1(new boolean[] { false, false, false, false, true }),
        POS_2(new boolean[] { false, false, false, true, true }),
        POS_3(new boolean[] { false, false, true, true, true }),
        POS_4(new boolean[] { false, true, true, true, true }),
        POS_5(new boolean[] { true, true, true, true, true });

        public boolean[] thropperState;

        private ThropperState(boolean[] thropperState) {
            this.thropperState = thropperState;
        }
    }

    public LazyCANSparkMax INTAKE_MOTOR, HOPPER_FEEDER_MOTOR, CORNER_MOTOR, SHOOTER_FEEDER_MOTOR, GATEKEEPER_MOTOR;
    public CANEncoder HOPPER_FEEDER_ENCODER, CORNER_ENCODER, SHOOTER_FEEDER_ENCODER;
    public DigitalInput SENSOR_0, SENSOR_1, SENSOR_2;

    boolean[] ballArray = { false, false, false, false, false };
    private static final Thropper instance = new Thropper();

    private boolean inPlaceEntry = false;
    private boolean inPlaceExit = false;
    private boolean isMoving;

    private DigitalInput[] sensorPositions = { SENSOR_0, SENSOR_1, SENSOR_2 }; // top to bottom //0 = 1.5, 1 = 3, 2 = 4 mathhh

    public static Thropper getInstance() {
        return instance;
    }

    private Thropper() {
        configMotors();
        configEncoders();
        configSensors();
    }

    // public void countBalls(DigitalInput entrySensor, DigitalInput exitSensor) {
    // if (inPlaceEntry == false && entrySensor.get()) {
    // countBalls++;
    // System.out.println("Number of balls: " + countBalls);
    // inPlaceEntry = true;
    // } else if (!entrySensor.get()) {
    // inPlaceEntry = false;
    // }

    // if (inPlaceExit == false && exitSensor.get()) {
    // countBalls--;
    // System.out.println("Number of balls: " + countBalls);
    // inPlaceExit = true;
    // } else if (!exitSensor.get()) {
    // inPlaceExit = false;
    // }
    // }

    @Override
    public void update() {
        if (!isMoving) {
            checkSensors(); //TODO: do we need this?
        }
        // countBalls(SENSOR_0, SENSOR_4);
    }

    /*private void checkSensors() {
        ballArray[0] = SENSOR_0.get();
        ballArray[1] = SENSOR_1.get();
        ballArray[2] = SENSOR_2.get();
        ballArray[3] = SENSOR_3.get();
        ballArray[4] = SENSOR_4.get();
    }*/
    private void checkSensors() {
        
    }

    private void configMotors() {
        INTAKE_MOTOR = new LazyCANSparkMax(Constants.IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
        HOPPER_FEEDER_MOTOR = new LazyCANSparkMax(Constants.HopperConstants.HOPPER_FEEDER_MOTOR_ID,
                MotorType.kBrushless);
        CORNER_MOTOR = new LazyCANSparkMax(Constants.HopperConstants.CORNER_MOTOR_ID, MotorType.kBrushless);
        SHOOTER_FEEDER_MOTOR = new LazyCANSparkMax(Constants.HopperConstants.SHOOTER_FEEDER_MOTOR_ID,
                MotorType.kBrushless);
        GATEKEEPER_MOTOR = new LazyCANSparkMax(Constants.HopperConstants.GATEKEEPER_MOTOR_ID, MotorType.kBrushless);
    }

    private void configEncoders() {
        HOPPER_FEEDER_ENCODER = HOPPER_FEEDER_MOTOR.getEncoder();
        CORNER_ENCODER = CORNER_MOTOR.getEncoder();
        SHOOTER_FEEDER_ENCODER = SHOOTER_FEEDER_MOTOR.getEncoder();
    }

    private void configSensors() {
        SENSOR_0 = new DigitalInput(Constants.HopperConstants.SENSOR_0_ID);
        SENSOR_1 = new DigitalInput(Constants.HopperConstants.SENSOR_1_ID);
        SENSOR_2 = new DigitalInput(Constants.HopperConstants.SENSOR_2_ID);
    }

    public void setBallOrder(boolean[] CurrBallArray) {
        if (CurrBallArray.length == ballArray.length) {
            int tempI = 0;
            for (boolean element : ballArray) {
                element = CurrBallArray[tempI];
                tempI++;
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
        return (getNumBalls() == Constants.HopperConstants.CAPACITY);
    }

    private boolean[] shift(boolean[] in_array) {
        boolean[] out_array = new boolean[in_array.length];
        for (int i = 0; i < in_array.length - 1; i++) {
            out_array[i] = in_array[i + 1];
        }
        out_array[in_array.length - 1] = false;
        return out_array;
    }

    private boolean[] addBall(boolean[] in_array) {
        boolean[] out_array = new boolean[in_array.length];
        for (int i = 0; i < in_array.length; i++) {
            boolean added = false;
            if(in_array[i]) {
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

    public void setIsMoving(boolean isMoving) {
        this.isMoving = isMoving;
    }

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

    public void setMotorPowers(double p_Gatekeeer, double p_ShooterFeeder, double p_Corner, double p_HopperFeeder) {
        GATEKEEPER_MOTOR.set(p_Gatekeeer);
        SHOOTER_FEEDER_MOTOR.set(p_ShooterFeeder);
        CORNER_MOTOR.set(p_Corner);
        HOPPER_FEEDER_MOTOR.set(p_HopperFeeder);
    }
}