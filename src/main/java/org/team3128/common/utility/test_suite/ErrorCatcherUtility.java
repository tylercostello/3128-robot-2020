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
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import org.team3128.common.hardware.motor.LazyTalonFX;

/**
 * Utility used to catch breaks in the CAN chain
 * This code is clinically bald
 * 
 * author Tyler Costello, Daniel Wang, Jude T. Lifset
 * 
 */
public class ErrorCatcherUtility {
    public CanDevices[] CanChain = new CanDevices[42];
    public static ErrorCode errorCode;
    public CanDevices lastDevice;

    /*public TalonSRX talon;
    public VictorSPX victor;
    public PowerDistributionPanel pdp;*/
    private double pdpTemp;
    private double sparkTemp;
    public CANError canError;
    
    

    public ErrorCatcherUtility(CanDevices[] CanChain){
      this.CanChain = CanChain;  
    }

    public void ErrorCatcher(){

        //Iterates over each CAN device in the chain, in order, and checks if it is good
        errorCode=ErrorCode.OK;

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
                    errorCode = ErrorCode.RxTimeout;
                } else{
                    errorCode=ErrorCode.OK;
                }
            }

            else if (device.type==CanDevices.DeviceType.FALCON){
                errorCode = device.falcon.configRemoteFeedbackFilter(device.id, RemoteSensorSource.CANifier_Quadrature,0, 10);

            }

            else if (device.type==CanDevices.DeviceType.PDP){

                pdpTemp=device.pdp.getTemperature();
                Log.info("ErrorCatcher", "PDP temp "+pdpTemp);
                if (pdpTemp < 5){
                    errorCode = ErrorCode.RxTimeout;
                }
            }
            



            //If the current CAN device is not good, log it
            if(errorCode != ErrorCode.OK){
                if(device == CanChain[0]){
                    Log.info("ErrorCatcher", "RoboRIO to " +device.name+ " " + device.id +" CAN wire is disconnected");
                    NarwhalDashboard.put("ErrorCatcher", "RoboRIO to " +device.name+ " " + device.id +" CAN wire is disconnected");
                }
                else{
                    Log.info("ErrorCatcher", lastDevice.name + " " + lastDevice.id + " to " +device.name+ " " + device.id +" CAN wire is disconnected");
                    NarwhalDashboard.put("ErrorCatcher", lastDevice.name + " " + lastDevice.id + " to " +device.name+ " " + device.id +" CAN wire is disconnected");

                }
                break;
            } else{
               NarwhalDashboard.put("ErrorCatcher", "No Errors");
            }

            lastDevice = device; 
            //Used because we need the last CAN device in the chain to find which 2 CAN devices are connected by the bad chain

        }
    }
}