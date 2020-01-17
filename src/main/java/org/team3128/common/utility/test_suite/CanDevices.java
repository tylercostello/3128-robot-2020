package org.team3128.common.utility.test_suite;

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

    public CanDevices(DeviceType type, int id, String name){

        this.type = type;
        this.id = id;
        this.name = name;

    }

   

}