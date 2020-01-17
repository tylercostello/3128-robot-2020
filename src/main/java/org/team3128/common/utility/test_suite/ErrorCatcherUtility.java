package org.team3128.common.utility.test_suite;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import org.team3128.common.utility.Log;


import org.team3128.common.utility.test_suite.CanDevices;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

/**
 * This is the class that catches errors with wiring.
 * 
 * @author Tyler Costello, Daniel Wang, Jude T. Lifset
 *
 */
public class ErrorCatcherUtility {
    public CanDevices[] CanChain = new CanDevices[42];
    public ErrorCode errorCode;
    public CanDevices lastDevice;

    public TalonSRX talon;
    public VictorSPX victor;
    public PowerDistributionPanel pdp;
    private double pdpTemp;
    
    //TODO:
    //public SparkMax and public Falcon

    public ErrorCatcherUtility(CanDevices[] CanChain){
      this.CanChain = CanChain;  
    }

    public void ErrorCatcher(){

        //Iterates over each CAN device in the chain, in order, and checks if it is good
        for(CanDevices device : CanChain){
            if (device == null){
                break;
            }
            if(device.type == CanDevices.DeviceType.TALON){
                talon=new TalonSRX(device.id);
                errorCode = talon.configRemoteFeedbackFilter(device.id, RemoteSensorSource.CANifier_Quadrature,0, 10);
            }
            else if (device.type==CanDevices.DeviceType.VICTOR){
                victor = new VictorSPX(device.id);
                errorCode = victor.configRemoteFeedbackFilter(device.id, RemoteSensorSource.CANifier_Quadrature,0, 10);

            }
            else if (device.type==CanDevices.DeviceType.PDP){
                pdp = new PowerDistributionPanel(device.id);
                pdpTemp=pdp.getTemperature();
                if (pdpTemp < -30){
                    errorCode = ErrorCode.RxTimeout;
                }
            }
            //TODO:
            //Add functionality for other CAN devices

            //If the current CAN device is not good, log it
            if(errorCode != ErrorCode.OK){
                if(lastDevice == null){
                    Log.info("ErrorCatcher", "RoboRIO to " +device.name+ " " + device.id +" CAN wire is disconnected");
                }
                else{
                    Log.info("ErrorCatcher", lastDevice.name + " " + lastDevice.id + " to " +device.name+ " " + device.id +" CAN wire is disconnected");
                }
                break;
            }

            lastDevice = device; 
            //Used because we need the last CAN device in the chain to find which 2 CAN devices are connected by the bad chain

        }
    }
}