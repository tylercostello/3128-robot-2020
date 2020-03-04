package org.team3128.compbot.subsystems;

import org.team3128.common.utility.Log;

import edu.wpi.first.wpilibj.DigitalInput;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.team3128.common.generics.Threaded;
import org.team3128.common.hardware.motor.LazyCANSparkMax;
import org.team3128.common.hardware.motor.LazyTalonSRX;
import org.team3128.common.hardware.motor.LazyVictorSPX;
import org.team3128.common.utility.enums.Direction;


public class Climber extends Threaded {

    public static final Climber instance = new Climber();
    public LazyVictorSPX LEFT_MOTOR, RIGHT_MOTOR;
    public boolean isClimbing = false;

    public static Climber getInstance() {
        return instance;
    }

    private Climber() {
        configMotors();
    }

    private void configMotors() {
        LEFT_MOTOR = new LazyVictorSPX(Constants.ClimberConstants.LEFT_MOTOR_ID);
        RIGHT_MOTOR = new LazyVictorSPX(Constants.ClimberConstants.RIGHT_MOTOR_ID);
    }

    public void setPower(double power){
        if (isClimbing) {
            LEFT_MOTOR.set(ControlMode.PercentOutput, power);
            RIGHT_MOTOR.set(ControlMode.PercentOutput, -power);
        } else {
            LEFT_MOTOR.set(ControlMode.PercentOutput, 0);
            RIGHT_MOTOR.set(ControlMode.PercentOutput, 0);
        }
    }

    public void setIsClimbing(boolean value) {
        isClimbing = value;
    }

    @Override
    public void update() {
        // nothing needed here
    }

}