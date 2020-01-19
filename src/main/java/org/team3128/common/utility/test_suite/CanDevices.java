package org.team3128.common.utility.test_suite;

//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import org.team3128.common.hardware.motor.LazyTalonSRX;

import edu.wpi.first.wpilibj.PowerDistributionPanel;


/**
 * This class is used to create CAN device objects.
 * 
 * @author Tyler Costello, Daniel Wang, Jude T. Lifset
 *
 */
public class CanDevices{
    
    public enum DeviceType
    { 
        VICTOR,
        TALON,
        SPARKMAX,
        FALCON,
        PDP;
    } 

    public DeviceType type;
    public int id;
    public String name;
    public LazyTalonSRX talon;
    public VictorSPX victor;
    public PowerDistributionPanel pdp;

    public CanDevices(DeviceType type, int id, String name, LazyTalonSRX talon, VictorSPX victor, PowerDistributionPanel pdp){
        this.type = type;
        this.id = id;
        this.name = name;
        this.talon = talon;
        this.victor = victor;
        this.pdp = pdp;

    }

   

}