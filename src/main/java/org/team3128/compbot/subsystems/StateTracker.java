package org.team3128.compbot.subsystems;

import org.team3128.common.generics.Threaded;
import org.team3128.common.utility.Log;
import org.team3128.compbot.subsystems.Arm.ArmState;
import org.team3128.compbot.subsystems.Shooter.ShooterState;


public class StateTracker{
    public enum RobotState {
        SHORT_RANGE(Arm.ArmState.SHORT_RANGE, Shooter.ShooterState.SHORT_RANGE),
        MID_RANGE(Arm.ArmState.MID_RANGE, Shooter.ShooterState.MID_RANGE),
        LONG_RANGE(Arm.ArmState.LONG_RANGE, Shooter.ShooterState.LONG_RANGE);

        public ArmState targetArmState;
        public ShooterState targetShooterState;

        private RobotState(final ArmState armState, final ShooterState shooterState) {
            this.targetArmState = armState;
            this.targetShooterState = shooterState;
        }
        
    }
    
    private static StateTracker instance = null;
    public static RobotState robotState;

    public static void initialize(){
        instance = new StateTracker();
    }

    public static StateTracker getInstance(){
        if (instance != null) {
            return instance;
        }

        Log.fatal("StateTracker", "Attempted to get instance before initialization! Call initialize(...) first.");
        return null;
    }

    private StateTracker(){
        robotState = RobotState.MID_RANGE;
    }

    public void setState(final RobotState desiredState) {
        robotState = desiredState;
    }

    public RobotState getState() {
        return robotState;
    }
}