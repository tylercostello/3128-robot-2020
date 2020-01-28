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
import com.revrobotics.CANSparkMax;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

import org.team3128.common.hardware.limelight.Limelight;
import org.team3128.common.hardware.limelight.LimelightKey;
import org.team3128.common.hardware.motor.LazyTalonFX;

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
    public double maxVelocity;

    /*public TalonSRX talon;
    public VictorSPX victor;
    public PowerDistributionPanel pdp;*/
    private double pdpTemp;
    private double sparkTemp;
    public CANError canError;
    
    

    public ErrorCatcherUtility(CanDevices[] CanChain, Limelight[] limelights){
      this.CanChain = CanChain;  
      this.limelights = limelights;
      //this.maxVelocity = maxVelocity;
    }

    public void ErrorCatcher(){

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
              /*  canError=device.spark.setIdleMode(CANSparkMax.IdleMode.kCoast);
                if (canError != CANError.kOk){
                    Log.info("ErrorCatcher", "bad"); 
                }
                else{
                    Log.info("ErrorCatcher", "ok");
                }*/
                /*if (device.spark.setCANTimeout(10) != CANError.kOk){
                    errorCode = ErrorCode.RxTimeout;
                }*/

              /*  canError=device.spark.setCANTimeout(10);
                if (canError==CANError.kOk){
                    Log.info("ErrorCatcher", "ok");
                }
                else
                {
                    Log.info("ErrorCatcher", ""+canError);
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
    /*
    public void velocityTester(double maxVelocity) {
        for(CanDevices device : CanChain){
            if (device.type == CanDevices.DeviceType.TALON && device.use == CanDevices.DeviceUse.LEADER){
                device.talon.set(ControlMode.Velocity, maxVelocity);
                int plateauCount = 0;
                while (plateauCount<5){
                    if (device.talon.getSelectedSensorVelocity() == maxVelocity)
                        plateauCount++;
                }
                device.talon.set(ControlMode.Velocity, 0);
            }
            if (device.type == CanDevices.DeviceType.SPARK && device.use == CanDevices.DeviceUse.LEADER){
               // device.spark.get();
                int plateauCount = 0;
                while (plateauCount<5){
                    if (device.talon.getSelectedSensorVelocity() == maxVelocity)
                        plateauCount++;
                }
            }
            if (device.type == CanDevices.DeviceType.FALCON && device.use == CanDevices.DeviceUse.LEADER){
                device.falcon.set(ControlMode.Velocity, maxVelocity);
                int plateauCount = 0;
                while (plateauCount<5){
                    if (device.talon.getSelectedSensorVelocity() == maxVelocity)
                        plateauCount++;
                }
            }
        }
    }
    */
    public void testEverything() {
        ErrorCatcher();
        limelightCheck();
        //velocityTester(maxVelocity);
    }
}