package team3176.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.constants.ArmConstants;

public class Elbow extends SubsystemBase {

private static Elbow instance;

private DigitalInput pickupLimiter;
private DigitalInput floorLimiter;
private TalonFX elbowMotor;
private ElbowState currentState;
private String currentStateString;
private int currentPosition;

/* Discrete positions for the arm */
/* This will likely need to be defined elsewhere so that callers have access to this type
 * in order to command the arm to the desired state.
 */
public enum ElbowState {
    UNKNOWN,
    PICKUP,
    HIGH,
    MID,
    FLOOR,
    TRANSITIONING   //Optional state that is currently not used
}

public Elbow() {
    pickupLimiter = new DigitalInput(ArmConstants.ELBOW_PICKUP_LIMIT_CHAN);
    floorLimiter = new DigitalInput(ArmConstants.ELBOW_FLOOR_LIMIT_CHAN);
    elbowMotor = new TalonFX(ArmConstants.ELBOW_FALCON_CAN_ID);

    currentState = ElbowState.UNKNOWN;
    currentStateString = "Unknown";
    currentPosition = 0;

    System.out.println("Elbow Motor has been constructed");

    elbowMotor.configFactoryDefault();
    elbowMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, ArmConstants.kPIDLoopIndex, ArmConstants.kTimeoutMS);
    elbowMotor.configAllowableClosedloopError(0, ArmConstants.kPIDLoopIndex, ArmConstants.kTimeoutMS);
    elbowMotor.setSensorPhase(true);
    elbowMotor.configClosedloopRamp(ArmConstants.kRampRate, ArmConstants.kTimeoutMS);
    
    elbowMotor.config_kP(0, ArmConstants.Elbow_PIDFConstants[0][0]);
    elbowMotor.config_kI(0, ArmConstants.Elbow_PIDFConstants[0][1]);
    elbowMotor.config_kD(0, ArmConstants.Elbow_PIDFConstants[0][2]);
    elbowMotor.config_kF(0, ArmConstants.Elbow_PIDFConstants[0][3]);
    elbowMotor.config_IntegralZone(0, ArmConstants.Elbow_PIDFConstants[0][4]);

    elbowMotor.config_kP(0, ArmConstants.Elbow_PIDFConstants[0][0]);
    elbowMotor.config_kI(0, ArmConstants.Elbow_PIDFConstants[0][1]);
    elbowMotor.config_kD(0, ArmConstants.Elbow_PIDFConstants[0][2]);
    elbowMotor.config_kF(0, ArmConstants.Elbow_PIDFConstants[0][3]);
    elbowMotor.config_IntegralZone(0, ArmConstants.Elbow_PIDFConstants[0][4]);

}

/* Set the desired discrete position of the elbow */
public void setElbowState(ElbowState targetState ) {

    switch (targetState) {
        case PICKUP:
            elbowMotor.set(ControlMode.Position, ArmConstants.PICKUP_POSITION);
            currentState = targetState;
            currentStateString = "Pickup";
            break;
        case HIGH:
            elbowMotor.set(ControlMode.Position, ArmConstants.HIGH_POSITION);
            currentState = targetState;
            currentStateString = "High";
            break;
        //TODO: Add the last 2 additional discrete positions
        default:
            currentStateString = "Invalid";
            break; 
    
    }

    SmartDashboard.putString("Elbow Position", currentStateString);  

}

public ElbowState getElbowState() {

    return currentState;
}

/* Returns the position of the elbow */
public int getElbowPosition() {
//TOOD: Look at the API on how to return the position
    return currentPosition;
}

public static Elbow getInstance() {
    if(instance == null) {instance = new Elbow();}
    return instance;
  }


}



