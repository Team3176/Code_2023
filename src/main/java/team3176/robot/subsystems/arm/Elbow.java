package team3176.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import team3176.robot.constants.ArmConstants;

public class Elbow {
    
private DigitalInput pickupLimiter;
private DigitalInput floorLimiter;
private TalonFX elbowMotor;
private ElbowState currentState;
private int currentPosition;

/* Discrete positions for the arm */
public enum ElbowState {
    UNKNOWN,
    PICKUP,
    HIGH,
    MID,
    FLOOR,
    TRANSITIONING
}

public Elbow() {
    pickupLimiter = new DigitalInput(ArmConstants.ELBOW_PICKUP_LIMIT_CHAN);
    floorLimiter = new DigitalInput(ArmConstants.ELBOW_FLOOR_LIMIT_CHAN);
    
    elbowMotor = new TalonFX(ArmConstants.ARM_ELBOW_FALCON_CAN_ID);
    currentState = ElbowState.UNKNOWN;
    currentPosition = 0;
}

/* Set the desired discrete position of the elbow */
public void setElbowState(ElbowState targetState ) {


}

public ElbowState getElbowState() {

    return currentState;
}

/* Returns the position of the elbow */
public int getElbowPosition() {

    return currentPosition;
}



}



