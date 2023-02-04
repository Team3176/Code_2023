package team3176.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import team3176.robot.constants.ArmConstants;

public class Elbow {
    
private DigitalInput intakeLimiter;
private DigitalInput extendLimiter;
private TalonFX elbowMotor;


public Elbow() {
intakeLimiter = new DigitalInput(ArmConstants.ELBOW_INTAKE_LIMIT);
extendLimiter = new DigitalInput(ArmConstants.ELBOW_EXTEND_LIMIT);
elbowMotor = new TalonFX(ArmConstants.ARM_ELBOW_FALCON_CAN_ID);
}




}



