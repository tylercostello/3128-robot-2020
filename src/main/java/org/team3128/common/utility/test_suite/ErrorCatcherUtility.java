package org.team3128.common.utility.test_suite;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;

import org.team3128.common.narwhaldashboard.NarwhalDashboard;
import org.team3128.common.utility.Log;

import org.team3128.common.utility.test_suite.CanDevices;

import com.revrobotics.CANError;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

import org.team3128.common.hardware.limelight.Limelight;
import org.team3128.common.hardware.limelight.LimelightKey;
import org.team3128.common.hardware.motor.LazyTalonFX;
import edu.wpi.first.wpilibj.Timer;
import org.team3128.athos.main.MainAthos;
import org.team3128.common.drive.DriveSignal;
import org.team3128.athos.subsystems.*;
/**
 * Utility used to catch breaks in the CAN chain
 * 
 * 
 * author Tyler Costello, Daniel Wang, Jude T. Lifset
 * 
 */
public class ErrorCatcherUtility {
    public CanDevices[] CanChain = new CanDevices[42];
    public Limelight[] limelights = new Limelight[5];
    public static ErrorCode errorCode;
    public CanDevices lastDevice;
    public CANEncoder encoder;

    /*public TalonSRX talon;
    public VictorSPX victor;
    public PowerDistributionPanel pdp;*/
    private double pdpTemp;
    private double sparkTemp;
    public CANError canError;
    public boolean forwardWorks;
    public boolean backwardWorks;

    //public NEODrive neoDrive;
    
    

    public ErrorCatcherUtility(CanDevices[] CanChain, Limelight[] limelights){
      this.CanChain = CanChain;  
      this.limelights = limelights;
    }

    public void ErrorCatcher(){
        //neoDrive=MainAthos.drive;
        //Iterates over each CAN device in the chain, in order, and checks if it is good
        errorCode=ErrorCode.OK;
        //bridge checker
        NarwhalDashboard.put("ErrorCatcherBridge", "Bridge is connected");
        for(CanDevices device : CanChain){
            if (device == null){
                break;
            }
            if(device.type == CanDevices.DeviceType.TALON){

                errorCode = device.talon.configRemoteFeedbackFilter(device.id, RemoteSensorSource.CANifier_Quadrature,0, 10);
            }
            else if (device.type==CanDevices.DeviceType.VICTOR){
                errorCode = device.victor.configRemoteFeedbackFilter(device.id, RemoteSensorSource.CANifier_Quadrature,0, 10);

            }
            else if (device.type==CanDevices.DeviceType.SPARK){
               // canError=device.spark.setCANTimeout(10);
               /* canError=device.spark.setIdleMode(CANSparkMax.IdleMode.kCoast);
                if (canError != CANError.kOk){
                    Log.info("ErrorCatcher", "bad"); 
                }
                else{
                    Log.info("ErrorCatcher", "ok");
                }*/
                /*if (device.spark.setCANTimeout(10) != CANError.kOk){
                    errorCode = ErrorCode.RxTimeout;
                }*/
                /*
                canError=device.spark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 10);
                if (canError==CANError.kOk){
                    Log.info("ErrorCatcher", "ok");
                }
                else
                {
                    Log.info("ErrorCatcher", ""+canError);
                    errorCode=ErrorCode.SensorNotPresent;
                }*/
                
                sparkTemp=device.spark.getMotorTemperature();
                Log.info("ErrorCatcher", "Spark temp "+sparkTemp);

                if (sparkTemp < 5 || sparkTemp>100){
                    errorCode = ErrorCode.CAN_MSG_NOT_FOUND;
                } else if (device.spark.getEncoder() == null){
                    errorCode = ErrorCode.SensorNotPresent;
                } else {
                    errorCode=ErrorCode.OK;
                }
            }

            else if (device.type==CanDevices.DeviceType.FALCON){
                errorCode = device.falcon.configRemoteFeedbackFilter(device.id, RemoteSensorSource.CANifier_Quadrature,0, 10);
                Log.info("ErrorCatcher", "Falcons");
            }

            else if (device.type==CanDevices.DeviceType.PDP){

                pdpTemp=device.pdp.getTemperature();
                Log.info("ErrorCatcher", "PDP temp "+pdpTemp);
                if (pdpTemp < 5){
                    errorCode = ErrorCode.CAN_MSG_NOT_FOUND;
                }
            }

            //If the current CAN device is not good, log it
            if(errorCode != ErrorCode.OK){
                Log.info("ErrorCatcher", "ErrorCode: "+errorCode);
                if (errorCode == ErrorCode.CAN_MSG_NOT_FOUND || errorCode == ErrorCode.SigNotUpdated){
                    if(device == CanChain[0]){
                        Log.info("ErrorCatcher", "RoboRIO to " +device.name+ " " + device.id +" CAN wire is disconnected");
                        NarwhalDashboard.put("ErrorCatcherCAN", "RoboRIO to " +device.name+ " " + device.id +" CAN wire is disconnected");
                    }
                    else{
                        Log.info("ErrorCatcher", lastDevice.name + " " + lastDevice.id + " to " +device.name+ " " + device.id +" CAN wire is disconnected");
                        NarwhalDashboard.put("ErrorCatcherCAN", lastDevice.name + " " + lastDevice.id + " to " +device.name+ " " + device.id +" CAN wire is disconnected");
    
                    }
                    break;
                }
                if (errorCode == ErrorCode.SensorNotPresent){
                    Log.info("ErrorCatcher", device.name+ " " + device.id +" Encoder is disconnected");
                    NarwhalDashboard.put("ErrorCatcherEncoder", device.name+ " " + device.id +" Encoder is disconnected");
                }
                
            } else{
               NarwhalDashboard.put("ErrorCatcherCAN", "No CAN Errors");
            }

            lastDevice = device; 
            //Used because we need the last CAN device in the chain to find which 2 CAN devices are connected by the bad chain

        }
    }
    public void limelightCheck() {
        String limelightError = "";

        for(Limelight limelight : limelights){
            double tempLatency = limelight.getValue(LimelightKey.LATENCY, 5);
            Log.info("ErrorCatcher", "Limelight Latency: " + String.valueOf(tempLatency));

            if(tempLatency == 0){
                Log.info("ErrorCatcher", limelight.hostname + " is disconnected.");
                //limelightError += limelight.hostname + "is disconnected.\n";
                NarwhalDashboard.put("ErrorCatcherLimelight", limelight.hostname + " is disconnected.");  
            } else{
                Log.info("ErrorCatcher", limelight.hostname + " is connected.");
                //limelightError += limelight.hostname + "is connected.\n";
                NarwhalDashboard.put("ErrorCatcherLimelight", limelight.hostname + " is connected.");  
            }
        }
        //NarwhalDashboard.put("ErrorCatcherLimelight", limelightError);
    }
    
    public void velocityTester() {
        Log.info("ErrorCatcher", "Spark");
        int plateauCount = 0;
        double maxAchieved = 0;
        forwardWorks = false;
        backwardWorks = false;
        MainAthos.sparkDrive = new DriveSignal(Constants.TEST_SUITE_DRIVE_VELOCITY, Constants.TEST_SUITE_DRIVE_VELOCITY);
        MainAthos.drive.setWheelVelocity(MainAthos.sparkDrive);
        encoder=CanChain[0].spark.getEncoder();
        double time = Timer.getFPGATimestamp();
        double endTime = Timer.getFPGATimestamp();

        while (encoder.getVelocity() < 100 && (endTime-time) <= 4.2){
            
            if (maxAchieved<=encoder.getVelocity())
                maxAchieved=encoder.getVelocity();
            endTime = Timer.getFPGATimestamp();

           // Log.info("ErrorCatcher", "Time "+(endTime-time));
        }
        MainAthos.sparkDrive = new DriveSignal(0, 0);
        Log.info("ErrorCatcher", "ran");
        MainAthos.drive.setWheelVelocity(MainAthos.sparkDrive);
        if (maxAchieved > 4){
            forwardWorks = true;
        }
        Log.info("ErrorCatcher", "Max Velocity "+maxAchieved);
        Log.info("ErrorCatcher", "Movement works "+forwardWorks);

        MainAthos.sparkDrive = new DriveSignal(-Constants.TEST_SUITE_DRIVE_VELOCITY, -Constants.TEST_SUITE_DRIVE_VELOCITY);
        MainAthos.drive.setWheelVelocity(MainAthos.sparkDrive);
        encoder=CanChain[0].spark.getEncoder();
        time = Timer.getFPGATimestamp();
        endTime = Timer.getFPGATimestamp();
        maxAchieved = 0;
        while (encoder.getVelocity() > -100 && (endTime-time) <= 4.2){
            
            if (maxAchieved>=encoder.getVelocity())
                maxAchieved=encoder.getVelocity();
            endTime = Timer.getFPGATimestamp();

           // Log.info("ErrorCatcher", "Time "+(endTime-time));
        }
        MainAthos.sparkDrive = new DriveSignal(0, 0);
        Log.info("ErrorCatcher", "ran");
        MainAthos.drive.setWheelVelocity(MainAthos.sparkDrive);
        if (maxAchieved < -4){
            backwardWorks = true;
        }
        else {
            backwardWorks = false;
        }
        Log.info("ErrorCatcher", "Max Velocity "+maxAchieved);
        Log.info("ErrorCatcher", "Forward Movement works "+forwardWorks);
        Log.info("ErrorCatcher", "Backward Movement works "+backwardWorks);
        if (forwardWorks && backwardWorks){
            NarwhalDashboard.put("ErrorCatcherMovement", "Movement Works");
            Log.info("ErrorCatcher", "haha yes");  
        }
        else {
            NarwhalDashboard.put("ErrorCatcherMovement", "Movement Does Not Work");
            Log.info("ErrorCatcher", "haha no");  
        }
    }
    
    public void testEverything() {
        ErrorCatcher();
        ErrorCatcher();
        limelightCheck();
        velocityTester();
    }
}