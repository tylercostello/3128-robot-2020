package org.team3128.compbot.subsystems;

import org.team3128.common.utility.Log;

import edu.wpi.first.wpilibj.DigitalInput;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.team3128.common.generics.Threaded;
import org.team3128.common.hardware.motor.LazyCANSparkMax;
import org.team3128.common.hardware.motor.LazyTalonSRX;
import org.team3128.common.utility.enums.Direction;


public class Climber extends Threaded {

    public static final Climber instance = new Climber();
    public LazyTalonSRX ENGAGE_MOTOR, MOVE_MOTOR;
    public boolean isClimbing = false;

    public static Climber getInstance() {
        return instance;
    }

    private Climber() {
        configMotors();
    }

    private void configMotors() {
        ENGAGE_MOTOR = new LazyTalonSRX(Constants.ClimberConstants.ENGAGE_MOTOR_ID);
        //MOVE_MOTOR = new LazyTalonSRX(Constants.ClimberConstants.MOVE_MOTOR_ID);
    }

    /*public void balance(double power){
        if (isClimbing) {
            MOVE_MOTOR.set(ControlMode.PercentOutput, power);
        } else {
            MOVE_MOTOR.set(ControlMode.PercentOutput, 0.0);
        }
    }*/

    public void climb(double power){
        if (isClimbing) {
            ENGAGE_MOTOR.set(ControlMode.PercentOutput, power);
        } else {
            ENGAGE_MOTOR.set(ControlMode.PercentOutput, 0.0);
        }
    }

    public void setIsClimbing(boolean value){
        isClimbing = value;
    }

    @Override
    public void update() {
        // nothing needed here
    }

}













//no