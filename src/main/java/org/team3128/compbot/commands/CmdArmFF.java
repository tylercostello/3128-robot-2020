package org.team3128.compbot.commands;

import java.lang.invoke.WrongMethodTypeException;
import java.security.GuardedObject;
import java.util.Arrays;

import org.team3128.compbot.subsystems.Constants;
import org.team3128.compbot.subsystems.Arm;
import org.team3128.compbot.subsystems.Hopper.HopperState;

import edu.wpi.first.wpilibj.command.Command;

import edu.wpi.first.wpilibj.Timer;

import org.team3128.common.utility.Log;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.team3128.common.generics.Threaded;
import org.team3128.common.hardware.motor.LazyCANSparkMax;
import org.team3128.common.hardware.motor.LazyTalonFX;

public class CmdArmFF extends Command {

    Arm arm;

    public static double armOutput = 0;
    public static double currentAngle, previousAngle;
    public static double increment = 0.05;    
    public static int counter = 0;
    //public static double previousAngle = 0;
    public static double sumBatteryVoltage, sumAngle, sumBusVoltage, startVoltage, currentTime, pastTime;

    int plateauCount = 0;


    public CmdArmFF(Arm arm) {
        this.arm = arm;
    }

    @Override
    protected void initialize() {
        previousAngle = arm.getAngle();
        pastTime = Timer.getFPGATimestamp();
    }

    @Override
    protected void execute() {

        arm.ARM_MOTOR_LEADER.set(ControlMode.PercentOutput, armOutput);

        currentTime = Timer.getFPGATimestamp();

        currentAngle = arm.getAngle();

        if (Math.abs((previousAngle - currentAngle) / (pastTime - currentTime)) <= 0.001){
            counter += 1;

            sumBatteryVoltage += RobotController.getBatteryVoltage(); 
            sumAngle += currentAngle;
            sumBusVoltage += arm.ARM_MOTOR_LEADER.getBusVoltage();

            if (counter >= 50) {
                Log.info("Shooter", "Current armOutput/Percent armOutput: " + armOutput + " Average Angle: " + (sumAngle/counter) + " Average Voltage Battery: " + (sumBatteryVoltage/counter) + " Average Voltage Bus: " + (sumBusVoltage/counter)); 
                armOutput += increment;
                counter = 0;
                sumBatteryVoltage = 0; 
                sumAngle = 0; 
                sumBusVoltage = 0;
            }
        }
        
        if (arm.LIMIT_SWITCH.get()) {
            Log.info("Shooter", "Reached armOutput, starting downwards loop");
            increment = -0.05;
        }

        previousAngle = currentAngle;
        
    }

    @Override
    protected boolean isFinished() {
        if ((armOutput == 0) && (increment == -0.05)) {
            Log.info("Shooter", "Finished with loop");
            armOutput = 0;
            increment = 0;
            return true;
        } else {
            return false;
        }
    }

    @Override
    protected void end() {
        arm.ARM_MOTOR_LEADER.set(ControlMode.PercentOutput, 0);
    }

    @Override
    protected void interrupted() {
        end();
    }

}