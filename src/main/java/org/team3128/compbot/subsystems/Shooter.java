/** 
 * @author Jude, Tyler, Daniel
 */
package org.team3128.compbot.subsystems;

import org.team3128.common.hardware.motor.LazyCANSparkMax;
import org.team3128.common.generics.Threaded;
import org.team3128.common.control.RateLimiter;
import org.team3128.common.control.AsynchronousPid;
import org.team3128.common.control.motion.RamseteController;
import org.team3128.common.control.trajectory.Trajectory;
import org.team3128.common.control.trajectory.Trajectory.State;
import org.team3128.common.drive.AutoDriveSignal;
import org.team3128.common.drive.DriveSignal;
import org.team3128.common.utility.math.Rotation2D;
import org.team3128.common.utility.NarwhalUtility;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import org.team3128.common.hardware.motor.LazyCANSparkMax;
import org.team3128.common.utility.RobotMath;

import edu.wpi.first.wpilibj.Timer;

import org.team3128.common.utility.Log;
import org.team3128.common.drive.Drive;

public class Shooter extends Threaded {

    public double targetVelocity;

    public Shooter(double targetVelocity){

    }

    @Override
    public void update(LazyCANSparkMax shooter, double targetVelocity, double kP) {
        double currentVelocity = shooter.getEncoder().getVelocity();
        double error = targetVelocity - currentVelocity;
        shooter.set(shooter.get()+error*kP);
        

    }

}