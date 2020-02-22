package org.team3128.compbot.subsystems;

import org.team3128.common.utility.Log;

import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.team3128.common.generics.Threaded;
import org.team3128.common.hardware.motor.LazyCANSparkMax;
import edu.wpi.first.wpilibj.Servo;

public class Climber extends Threaded {

    public static final Climber instance = new Climber();
    public Servo leftClimbServo;
    public Servo rightClimbServo;
    LazyCANSparkMax INTAKE_MOTOR;

    public static Climber getInstance() {
        return instance;
    }

    private Climber() {
        configMotors();
    }

    private void configMotors() {
        leftClimbServo = new Servo(Constants.ClimberConstants.LEFT_SERVO_ID);
		rightClimbServo = new Servo(Constants.ClimberConstants.RIGHT_SERVO_ID);
    }

    public void engageClimber() {
        leftClimbServo.setAngle(Constants.ClimberConstants.LEFT_ENGAGE_ANGLE);
        rightClimbServo.setAngle(Constants.ClimberConstants.RIGHT_ENGAGE_ANGLE);
    }

    public void disengageClimber() {
        leftClimbServo.setAngle(Constants.ClimberConstants.LEFT_DISENGAGE_ANGLE);
        rightClimbServo.setAngle(Constants.ClimberConstants.RIGHT_DISENGAGE_ANGLE);
    }

    @Override
    public void update() {
        // nothing needed here
    }

}













//no