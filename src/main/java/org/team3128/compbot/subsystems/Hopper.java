package org.team3128.compbot.subsystems;

import org.team3128.common.utility.Log;
import org.team3128.compbot.subsystems.Arm.ArmState;

import edu.wpi.first.wpilibj.DigitalInput;

import org.team3128.common.generics.Threaded;
import org.team3128.common.hardware.motor.LazyCANSparkMax;
import org.team3128.common.hardware.motor.LazyVictorSPX;

import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.LinkedList;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;

import org.team3128.common.game_elements.Ball;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.RobotController;


public class Hopper extends Threaded {

    public enum HopperState {
        POS_0(new boolean[] { false, false, false, false }), POS_1(new boolean[] { false, false, false, true }),
        POS_2(new boolean[] { false, false, true, true }), POS_3(new boolean[] { false, true, true, true }),
        POS_4(new boolean[] { true, true, true, true });
        // POS_5(new boolean[] { true, true, true, true }),
        // POS_6(new boolean[] { false, true, true, true });

        public boolean[] hopperState;

        private HopperState(boolean[] hopperState) {
            this.hopperState = hopperState;
        }
    }

    public enum ActionState {
        STANDBY, INTAKING, SHOOTING, ORGANIZING, EJECTING;

        private ActionState() {

        }
    }

    public LazyCANSparkMax INTAKE_MOTOR, CORNER_MOTOR, GATEKEEPER_MOTOR;
    public LazyVictorSPX HOPPER_FEEDER_MOTOR;
    public CANEncoder HOPPER_FEEDER_ENCODER, CORNER_ENCODER;
    public DigitalInput SENSOR_0, SENSOR_1;

    public static Arm arm = Arm.getInstance();

    private static final Hopper instance = new Hopper();

    // private boolean isMoving;
    private boolean empty0 = true;
    private boolean empty1 = true;
    private boolean isFeeding = false;
    private boolean isLoading = false;
    private boolean isReversing = false;
    private double startTime;
    // private boolean isOrganizing = false;
    private boolean openTheGates = false;
    private double startPos = 0;
    public int ballCount;
    private int detectCount, notDetectCount, detectCount1, notDetectCount1;
    private boolean detectBool = false;
    private boolean detectBool1 = false;
    private boolean ejectBallInBottom = false; //needed so that we don't subtract from ballcount if there was a ball halfway in the hopper when ejecting

    public int openTheGatesCounter = 0;
    public int jamCount = 0;
    public int plateauCount = 0;
    public double shootingCornerPosition = 0;
    public boolean shootingReversingIndexer = true;

    public boolean SENSOR_0_STATE = false;
    public boolean SENSOR_1_STATE = false;

    public ActionState actionState;

    private DigitalInput[] sensorPositions = { SENSOR_0, SENSOR_1 }; // top to bottom //0 = 1.5, 1 = 3, 2 = 4
                                                                     // mathhh

    public static Hopper getInstance() {
        return instance;
    }

    private Hopper() {
        configMotors();
        configEncoders();
        configSensors();
        ballCount = 0; // TODO: initial ball count
    }

    @Override
    public void update() {

        SENSOR_0_STATE = detectsBall0();
        SENSOR_1_STATE = detectsBall1();

        //Log.info("Hopper", "detect count = " + detectCount);

        SmartDashboard.putBoolean("gatekeep", openTheGates);
        //SmartDashboard.putNumber("openTheGatesCounter", openTheGatesCounter);
        SmartDashboard.putBoolean("SENSOR_0_DETECT", SENSOR_0_STATE);
        SmartDashboard.putBoolean("SENSOR_1_DETECT", SENSOR_1_STATE);

        switch (actionState) {
            case STANDBY:
                if (!isFull() && !(SENSOR_0_STATE && (ballCount >= 3))) {
                    standbyIntake();
                } else {
                    setMotorPowers(0, 0, 0);
                    isFeeding = false;
                }
                break;

            case INTAKING:
                if (!isFull() && !(SENSOR_0_STATE && (ballCount >= 5))) {
                    standbyIntake();
                } else {
                    setMotorPowers(0, 0, 0);
                    isFeeding = false;
                }
                break;

            case SHOOTING:
                loadShoot();
                break;

            case ORGANIZING:
                organize();
                arm.setState(ArmState.STOWED);
                break;
            
            case EJECTING:
                eject();
                break;
        }

        SmartDashboard.putString("Hopper isFeeding", "" + isFeeding);
        if (SENSOR_1_STATE) {
            empty1 = false;
        } else {
            empty1 = true;
        }
        if (SENSOR_0_STATE) {
            empty0 = false;
        } else {
            empty0 = true;
        }
    }

    private void configMotors() {
        INTAKE_MOTOR = new LazyCANSparkMax(Constants.IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushed);
        HOPPER_FEEDER_MOTOR = new LazyVictorSPX(Constants.HopperConstants.HOPPER_FEEDER_MOTOR_ID);
        CORNER_MOTOR = new LazyCANSparkMax(Constants.HopperConstants.CORNER_MOTOR_ID, MotorType.kBrushless);
        GATEKEEPER_MOTOR = new LazyCANSparkMax(Constants.HopperConstants.GATEKEEPER_MOTOR_ID, MotorType.kBrushless);
    }

    private void configEncoders() {
        CORNER_ENCODER = CORNER_MOTOR.getEncoder();
    }

    private void configSensors() {
        SENSOR_0 = new DigitalInput(Constants.HopperConstants.SENSOR_0_ID);
        SENSOR_1 = new DigitalInput(Constants.HopperConstants.SENSOR_1_ID);
    }

    public boolean isReady() {
        return true; // TODO: only return true if there is a ball ready to be shot AND the shoot
                     // method isn't currently in use.
    }

    public boolean isEmpty() {
        return ballCount <= 0;
    }

    public boolean isFull() {
        return (ballCount == Constants.HopperConstants.CAPACITY - 1);
    }

    public boolean[] shift(boolean[] in_array) {
        boolean[] out_array = new boolean[in_array.length];
        for (int i = 0; i < in_array.length - 1; i++) {
            out_array[i] = in_array[i + 1];
        }
        out_array[in_array.length - 1] = false;
        return out_array;
    }

    public void setMotorPowers(double p_Gatekeeer, double p_Corner, double p_HopperFeeder) {
        GATEKEEPER_MOTOR.set(p_Gatekeeer);
        CORNER_MOTOR.set(p_Corner);
        HOPPER_FEEDER_MOTOR.set(ControlMode.PercentOutput, p_HopperFeeder);
    }

    public void gateKeep(boolean value) {
        openTheGates = value;
    }

    // public void setIsOrganizing(boolean value) {
    // isOrganizing = value;
    // }

    public void shoot() {
        gateKeep(true);
    }

    public void unShoot() {
        gateKeep(false);
    }

    public boolean detectsBall0() {
        if(!SENSOR_0.get()) {
            detectCount++;
            notDetectCount = 0;
            //Log.info("Hopper", "sensor detects ball");
        } else {
            notDetectCount++;
            detectCount = 0;
            //Log.info("Hopper", "sensor doesn't detect ball");
        }
        if(detectCount >= 5) {
            detectCount = 0;
            notDetectCount = 0;
            detectBool = true;
        } else if (notDetectCount >= 5) {
            detectCount = 0;
            notDetectCount = 0;
            detectBool = false;
        }

        return detectBool;
        //return(!sensor.get());
    }

    public boolean detectsBall1() {
        if(!SENSOR_1.get()) {
            detectCount1++;
            notDetectCount1 = 0;
            //Log.info("Hopper", "sensor detects ball");
        } else {
            notDetectCount1++;
            detectCount1 = 0;
            //Log.info("Hopper", "sensor doesn't detect ball");
        }
        if(detectCount1 >= 7) {
            detectCount1 = 0;
            notDetectCount1 = 0;
            detectBool1 = true;
        } else if (notDetectCount1 >= 7) {
            detectCount1 = 0;
            notDetectCount1 = 0;
            detectBool1 = false;
        }

        return detectBool1;
        //return(!sensor.get());
    }

    public void setAction(ActionState state) {
        this.actionState = state;
        openTheGates = false;
        isLoading = false;
        isFeeding = false;
        if (state == ActionState.INTAKING) {
            INTAKE_MOTOR.set(Constants.IntakeConstants.INTAKE_MOTOR_ON_VALUE);
        } else if (state != ActionState.SHOOTING && state != ActionState.EJECTING) {
            INTAKE_MOTOR.set(Constants.IntakeConstants.INTAKE_MOTOR_OFF_VALUE);
        }
        if (state == ActionState.SHOOTING) {
            INTAKE_MOTOR.set(Constants.IntakeConstants.INTAKE_MOTOR_ON_VALUE / 2.5);
            shootingCornerPosition = CORNER_ENCODER.getPosition();
        }
        if (state == ActionState.EJECTING) {
            INTAKE_MOTOR.set(-Constants.IntakeConstants.INTAKE_MOTOR_OFF_VALUE);
            ejectBallInBottom = false;
        }
    }

    public void standbyIntake() { // run if we aren't saturated with balls and we are in INTAKING or STANDBY
                                  // states

        if (ballCount >= 3 && !isFeeding) {
            if (ballCount >= 5) {
                setMotorPowers(0, 0, 0);
            } else {
                setMotorPowers(0, 0, 0);
                // Log.info("Hopper", "Handling more than 3 balls");
            }
        } else if (SENSOR_1_STATE) { // if there is a ball in the first sensor position
            // empty1 = false; //tell code the position isn't empty
            if (!isFeeding) { // if we aren't trying to move the intaken ball into the lowest position
                setMotorPowers(0, Constants.HopperConstants.BASE_POWER, Constants.HopperConstants.INDEXER_POWER); // just
                                                                                                                  // move
                                                                                                                  // all
                                                                                                                  // the
                                                                                                                  // motors
            } else {
                setMotorPowers(0, Constants.HopperConstants.BASE_POWER, -Constants.HopperConstants.INDEXER_POWER); // otherwise, only move the corner motor
                                                                            // move the intaken ball into the lowest
                                                                            // position
            }
            //Log.info("Hopper", "detects ball");
        } else if (!SENSOR_1_STATE && !empty1) { // if there isn't a ball in the first position, but there was one in the last iteration
        // empty1 = true; //tell the code the position is empty
            Log.info("Hopper", "detected ball and was full previously, should iterate count if not reversing");
            if (!isReversing) {
                ballCount++; // iterate ballCount once because a ball has passed through our sensors
                Log.info("Hopper", "iterating ballCount 0");
            } else {
                Log.info("Hopper", "was reversing");
                isReversing = false;
            }

            startPos = CORNER_ENCODER.getPosition(); // record the current corner motor encoder position
            setMotorPowers(0, Constants.HopperConstants.BASE_POWER, -Constants.HopperConstants.INDEXER_POWER); // only move the corner motor to move the ball
                                                                        // into the lowest position
            // if (!isFull) {
            Log.info("Hopper", "setting isFeeding to true");
            isFeeding = true; // tell the code we are trying move this ball into the lowest position
            // }
        } else if (!isFeeding) {
            if (actionState == ActionState.INTAKING) {
                setMotorPowers(0, 0, Constants.HopperConstants.INDEXER_POWER);
            } else {
                //Log.info("Hopper", "alarm alarm alarm alarm alarm alarm alarm alarm");
                setMotorPowers(0, 0, 0);
            }
        } 
        if(isFeeding) { // if we are trying to move the ball into the lowest position
            setMotorPowers(0, Constants.HopperConstants.BASE_POWER, -Constants.HopperConstants.INDEXER_POWER); // only
                                                                                                               // move
                                                                                                               // the
                                                                                                               // corner
                                                                                                               // motor
            //////Log.info("Hopper", "" + CORNER_ENCODER.getPosition());
            if (ballCount > 0 && ballCount <= 4) {
                if (Math.abs(CORNER_ENCODER.getPosition() - Constants.HopperConstants.BALL_SPACING[ballCount - 1]) >= Math.abs(startPos)) { // if the ball gets to the right position
                    isFeeding = false; // we're done
                    Log.info("Hopper", "reached end of offset");
                }
            } else {
                Log.info("Hopper", "something was super wrong; your ballCount is " + ballCount);
                if (Math.abs(CORNER_ENCODER.getPosition() - Constants.HopperConstants.BALL_SPACING[0]) >= Math.abs(startPos)) { // if the ball gets to the right position
                    isFeeding = false; // we're done
                    Log.info("Hopper", "reached end of offset");
                }
            }
            //////Log.info("Hopper", "is feeding");
        }
    }

    public void loadShoot() { // loading the balls to be shot when in SHOOTING state
        if (SENSOR_0_STATE && !openTheGates) {
            setMotorPowers(0, 0, -Constants.HopperConstants.INDEXER_POWER / 1.5); // 0
        } else if (SENSOR_0_STATE && openTheGates) {
            //shootingCornerPosition = CORNER_ENCODER.getPosition();
            while (SENSOR_0_STATE) {
                SENSOR_0_STATE = detectsBall0();
                SENSOR_1_STATE = detectsBall1();
                if (Math.abs(CORNER_ENCODER.getPosition() - Constants.HopperConstants.BALL_SPACING[0]) >= Math.abs(shootingCornerPosition)) {
                    shootingReversingIndexer = false;
                }
                if (shootingReversingIndexer) {
                    setMotorPowers(Constants.HopperConstants.GATEKEEPER_POWER, Constants.HopperConstants.BASE_POWER,
                        -Constants.HopperConstants.INDEXER_POWER);
                    Log.info("Hopper", "Hopper is trying to hold 4th and 5th ball from entering the robot!");
                } else if (!shootingReversingIndexer) {
                    setMotorPowers(Constants.HopperConstants.GATEKEEPER_POWER, Constants.HopperConstants.BASE_POWER,
                        Constants.HopperConstants.INDEXER_POWER);
                }
                if (!SENSOR_1_STATE && !empty1) { // if there isn't a ball in the first position, but there was one in the last iteration
                    empty1 = true; //tell the code the position is empty
                    Log.info("Hopper", "detected ball and was full previously, should iterate count if not reversing");
                    if (!isReversing) {
                        ballCount++; // iterate ballCount once because a ball has passed through our sensors
                        Log.info("Hopper", "iterating ballCount 1");
                        shootingReversingIndexer = true;
                        shootingCornerPosition = CORNER_ENCODER.getPosition();
                    } else {
                        Log.info("Hopper", "was reversing");
                        isReversing = false;
                    }
                }
                if(SENSOR_1_STATE) {
                    //////Log.info("Hopper", "setting empty1 to false 1");
                    empty1 = false;
                } else {
                    //////Log.info("Hopper", "setting empty1 to true 1");
                    empty1 = true;
                }
            }
            ballCount--;
            //shootingCornerPosition = CORNER_ENCODER.getPosition();
            reloading:
            while (!SENSOR_0_STATE) {
                SENSOR_0_STATE = detectsBall0();
                SENSOR_1_STATE = detectsBall1();
                if (Math.abs(CORNER_ENCODER.getPosition() - Constants.HopperConstants.BALL_SPACING[0]) >= Math.abs(shootingCornerPosition)) {
                    shootingReversingIndexer = false;
                }
                if(actionState == ActionState.SHOOTING) {
                    if (shootingReversingIndexer) {
                        setMotorPowers(Constants.HopperConstants.GATEKEEPER_POWER, Constants.HopperConstants.BASE_POWER,
                            -Constants.HopperConstants.INDEXER_POWER);
                    } else if (!shootingReversingIndexer) {
                        setMotorPowers(Constants.HopperConstants.GATEKEEPER_POWER, Constants.HopperConstants.BASE_POWER,
                            Constants.HopperConstants.INDEXER_POWER);
                    }
                } else {
                    setMotorPowers(0, 0, 0);
                    break reloading;
                }
                if (!SENSOR_1_STATE && !empty1) { // if there isn't a ball in the first position, but there was one in the last iteration
                    empty1 = true; //tell the code the position is empty
                    Log.info("Hopper", "detected ball and was full previously, should iterate count if not reversing");
                    if (!isReversing) {
                        ballCount++; // iterate ballCount once because a ball has passed through our sensors
                        Log.info("Hopper", "iterating ballCount 2");
                        shootingReversingIndexer = true;
                        shootingCornerPosition = CORNER_ENCODER.getPosition();
                    } else {
                        Log.info("Hopper", "was reversing");
                        isReversing = false;
                    }
                }
                if(SENSOR_1_STATE) {
                    //////Log.info("Hopper", "setting empty1 to false 2");
                    empty1 = false;
                } else {
                    //////Log.info("Hopper", "setting empty1 to true 2");
                    empty1 = true;
                }
            }
            setMotorPowers(0, 0, -Constants.HopperConstants.INDEXER_POWER / 1.5); // 0
        } else if(openTheGates && !SENSOR_0_STATE){
            //shootingCornerPosition = CORNER_ENCODER.getPosition();
            loading:
            while (!SENSOR_0_STATE) {
                SENSOR_0_STATE = detectsBall0();
                SENSOR_1_STATE = detectsBall1();
                if (Math.abs(CORNER_ENCODER.getPosition() - Constants.HopperConstants.BALL_SPACING[0]) >= Math.abs(shootingCornerPosition)) {
                    shootingReversingIndexer = false;
                }
                if(actionState == ActionState.SHOOTING) {
                    if (shootingReversingIndexer) {
                        setMotorPowers(Constants.HopperConstants.GATEKEEPER_POWER, Constants.HopperConstants.BASE_POWER,
                            -Constants.HopperConstants.INDEXER_POWER);
                    } else if (!shootingReversingIndexer) {
                        setMotorPowers(Constants.HopperConstants.GATEKEEPER_POWER, Constants.HopperConstants.BASE_POWER,
                            Constants.HopperConstants.INDEXER_POWER);
                    }
                } else {
                    setMotorPowers(0, 0, 0);
                    break loading;
                }
                if (!SENSOR_1_STATE && !empty1) { // if there isn't a ball in the first position, but there was one in the last iteration
                    empty1 = true; //tell the code the position is empty
                    Log.info("Hopper", "detected ball and was full previously, should iterate count if not reversing");
                    if (!isReversing) {
                        ballCount++; // iterate ballCount once because a ball has passed through our sensors
                        Log.info("Hopper", "iterating ballCount 3");
                        shootingReversingIndexer = true;
                        shootingCornerPosition = CORNER_ENCODER.getPosition();
                    } else {
                        Log.info("Hopper", "was reversing");
                        isReversing = false;
                    }
                }
                if(SENSOR_1_STATE) {
                    //////Log.info("Hopper", "setting empty1 to false 3");
                    empty1 = false;
                } else {
                    //////Log.info("Hopper", "setting empty1 to true 3");
                    empty1 = true;
                }
            }
            setMotorPowers(0, 0, -Constants.HopperConstants.INDEXER_POWER / 1.5); // 0
        } else {
            setMotorPowers(0, 0, 0);
        }
    }

    public void organize() {
        if (isEmpty()) {
            setMotorPowers(0, 0, 0);
            setAction(ActionState.STANDBY);
            Log.info("Hopper", "ending organize because hopper is empty");
            setMotorPowers(0, 0, 0);
        } else {
            startTime = RobotController.getFPGATime();
            while(!SENSOR_1_STATE && (RobotController.getFPGATime() - startTime <= Constants.HopperConstants.REVERSE_TIMEOUT)) {
                //isReversing = true;
                SENSOR_1_STATE = detectsBall1();
                setMotorPowers(0, -Constants.HopperConstants.BASE_POWER, 0);
                Log.info("Hopper", "Reversing balls");
            }
            setMotorPowers(0, 0, 0);
            if(SENSOR_1_STATE) {
                isReversing = true;
            }
        }

        //setMotorPowers(0, 0, 0);
        setAction(ActionState.STANDBY);
    }

    public void eject() {
        if (!isEmpty()) {
            setMotorPowers(0, -Constants.HopperConstants.BASE_POWER, -Constants.HopperConstants.INDEXER_POWER);
        }
        if (SENSOR_1_STATE) {
            isReversing = true;
            if (empty1) { //if there wasn't a ball in the first position in the last iteration, but there is one now
                ejectBallInBottom = true; //then tell the hopper that there is a new ball in the bottom that is about to be ejected
            }
        } else {
            isReversing = false;
            ejectBallInBottom = false;
        }

        if (!SENSOR_1_STATE && !empty1) { // if there isn't a ball in the first position, but there was one in the last iteration
            empty1 = true; //tell the code the position is empty
            Log.info("Hopper", "(EJECTING) detected ball and was full previously, should de-iterate count");
            if (ejectBallInBottom) {
                ballCount--; // iterate ballCount once because a ball has passed through our sensors
                Log.info("Hopper", "de-iterating ballCount in EJECT");
            } else {
                Log.info("Hopper", "ejected ball that was halfway in the hopper and wasn't accounted for in ballCount");
            }
        }
    }

    public void setBallCount(int count) {
        this.ballCount = count;
    }

    public int getBallCount() {
        return this.ballCount;
    }
}