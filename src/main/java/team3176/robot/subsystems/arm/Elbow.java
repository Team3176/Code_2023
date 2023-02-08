package team3176.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.constants.ArmConstants;

public class Elbow extends SubsystemBase {

private static Elbow instance;

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

    System.out.println("Shoulder has been constructed");
    elbowMotor = new TalonFX(ArmConstants.ARM_SHOULDER_FALCON_CAN_ID);

    elbowMotor.configFactoryDefault();
    elbowMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, ArmConstants.kPIDLoopIndex, ArmConstants.kTimeoutMS);
    elbowMotor.configAllowableClosedloopError(0, ArmConstants.kPIDLoopIndex, ArmConstants.kTimeoutMS);
    elbowMotor.setSensorPhase(true);
    elbowMotor.configClosedloopRamp(ArmConstants.kRampRate, ArmConstants.kTimeoutMS);
    
    elbowMotor.config_kP(0, ArmConstants.PIDFConstants[0][0]);
    elbowMotor.config_kI(0, ArmConstants.PIDFConstants[0][1]);
    elbowMotor.config_kD(0, ArmConstants.PIDFConstants[0][2]);
    elbowMotor.config_kF(0, ArmConstants.PIDFConstants[0][3]);
    elbowMotor.config_IntegralZone(0, ArmConstants.PIDFConstants[0][4]);

    elbowMotor.config_kP(0, ArmConstants.PIDFConstants[0][0]);
    elbowMotor.config_kI(0, ArmConstants.PIDFConstants[0][1]);
    elbowMotor.config_kD(0, ArmConstants.PIDFConstants[0][2]);
    elbowMotor.config_kF(0, ArmConstants.PIDFConstants[0][3]);
    elbowMotor.config_IntegralZone(0, ArmConstants.PIDFConstants[0][4]);

}

/* Set the desired discrete position of the elbow */
public void setElbowState(ElbowState targetState ) {
/*
    switch (targetState) {
        ElbowState.HIGH:
    elbowMotor.set(ControlMode.Position, 0);
        SmartDashboard.putString("Scissor Lift", "Ground Level");  
        break;
    default:
        break; 
    
*/
}

public ElbowState getElbowState() {

    return currentState;
}

/* Returns the position of the elbow */
public int getElbowPosition() {

    return currentPosition;
}
public static Elbow getInstance() {
    if(instance == null) {instance = new Elbow();}
    return instance;
  }


}



