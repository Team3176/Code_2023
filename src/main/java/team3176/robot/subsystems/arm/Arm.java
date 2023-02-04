package team3176.robot.subsystems.arm;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import edu.wpi.first.networktables.DoublePublisher;

//import com.revrobotics.RelativeEncoder;
//import com.revrobotics.SparkMaxPIDController;
//import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;





public class Arm extends SubsystemBase {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    DoubleTopic Elbow =  inst.getDoubleTopic("/Networktable/Elbowdegrees");

    public final DoublePublisher elbowPub=Elbow.publish();
    private SparkMaxLimitSwitch m_forwardLimit;
    private SparkMaxLimitSwitch m_reverseLimit;
    //public JointSparkmax m_wrist;
    private static Arm instance;
    //private CANSparkMax m_motor = new CANSparkMax(0, null);
    
    
    public Arm() {
        System.out.println("Gobblygook");
        //m_wrist = new JointSparkmax(6);
        //m_reverseLimit = m_motor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
        //m_forwardLimit = m_motor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
    }

    private boolean getForwardLimitSwitch(){
        return m_forwardLimit.isPressed();
    }

    private boolean getReverseLimitSwitch() {
        return m_reverseLimit.isPressed();
    }

    @Override
    public void periodic(){
            elbowPub.set(1.0);

        //m_wrist.updatePID();
        //m_wrist.motorMove();
        //m_wrist.motorMovePercentOutput();
    }
    public static Arm getInstance() {
        if(instance == null) {instance = new Arm();}
        return instance;
      }
}