package org.team3128.compbot.subsystems;

import org.team3128.common.utility.Log;

import edu.wpi.first.wpilibj.DigitalInput;

import org.team3128.common.generics.Threaded;
import org.team3128.common.hardware.motor.LazyCANSparkMax;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.team3128.common.game_elements.Ball;

public class Hopper extends Threaded {

    public enum HopperState {
        DEBUG(0), AUTO(1);

        public double stateVal;

        private HopperState(double desiredVal) {
            this.stateVal = desiredVal;
        }
    }

    LazyCANSparkMax ALIGN_MOTOR, HOPPER_FEEDER_MOTOR, CORNER_MOTOR, SHOOTER_FEEDER_MOTOR;
    CANEncoder ALIGN_ENCODER, HOPPER_FEEDER_ENCODER, CORNER_ENCODER, SHOOTER_FEEDER_ENCODER;
    DigitalInput SENSOR_1, SENSOR_2, SENSOR_3, SENSOR_4;
    HopperState HOPPER_STATE;

    Ball[] ballArray;

    public static final Hopper instance = new Hopper();

    public static Hopper getInstance() {
        return instance;
    }

    private Hopper() {
        configMotors();
        configEncoders();
        configSensors();
        HOPPER_STATE = Constants.HopperConstants.HOPPER_STATE;
    }

    public void setState(HopperState desiredState) {
        HOPPER_STATE = desiredState;
    }

    @Override
    public void update() {
        if (HOPPER_STATE == HopperState.DEBUG) {
            ALIGN_MOTOR.set(Constants.HopperConstants.DEBUG_MOTOR_POWER);
            HOPPER_FEEDER_MOTOR.set(Constants.HopperConstants.DEBUG_MOTOR_POWER);
            CORNER_MOTOR.set(Constants.HopperConstants.DEBUG_MOTOR_POWER);
            SHOOTER_FEEDER_MOTOR.set(Constants.HopperConstants.DEBUG_MOTOR_POWER);
        }
    }

    private void configMotors() {
        ALIGN_MOTOR = new LazyCANSparkMax(Constants.HopperConstants.ALIGN_MOTOR_ID, MotorType.kBrushless);
        HOPPER_FEEDER_MOTOR = new LazyCANSparkMax(Constants.HopperConstants.HOPPER_FEEDER_MOTOR_ID,
                MotorType.kBrushless);
        CORNER_MOTOR = new LazyCANSparkMax(Constants.HopperConstants.CORNER_MOTOR_ID, MotorType.kBrushless);
        SHOOTER_FEEDER_MOTOR = new LazyCANSparkMax(Constants.HopperConstants.SHOOTER_FEEDER_MOTOR_ID,
                MotorType.kBrushless);
    }

    private void configEncoders() {
        ALIGN_ENCODER = ALIGN_MOTOR.getEncoder();
        HOPPER_FEEDER_ENCODER = HOPPER_FEEDER_MOTOR.getEncoder();
        CORNER_ENCODER = CORNER_MOTOR.getEncoder();
        SHOOTER_FEEDER_ENCODER = SHOOTER_FEEDER_MOTOR.getEncoder();
    }

    private void configSensors() {
        SENSOR_1 = new DigitalInput(Constants.HopperConstants.SENSOR_1_ID);
        SENSOR_2 = new DigitalInput(Constants.HopperConstants.SENSOR_2_ID);
        SENSOR_3 = new DigitalInput(Constants.HopperConstants.SENSOR_3_ID);
        SENSOR_4 = new DigitalInput(Constants.HopperConstants.SENSOR_4_ID);
    }

    public void shoot() {
    }

    public boolean isReady() {
        return false; // TODO: only return true if there is a ball ready to be shot AND the shoot
                      // method isn't currently in use.
    }

    public boolean isEmpty() {
        return false;
    }

}