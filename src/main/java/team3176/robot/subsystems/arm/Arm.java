package team3176.robot.subsystems.arm;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import com.revrobotics.RelativeEncoder;
//import com.revrobotics.SparkMaxPIDController;
//import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;



public class Arm extends SubsystemBase {
    public JointSparkmax m_wrist;
    private static Arm instance;


    private Arm() {
        m_wrist = new JointSparkmax(6);
    }

    @Override
    public void periodic(){
        m_wrist.updatePID();
        //m_wrist.motorMove();
        m_wrist.motorMovePercentOutput();
    }
    public static Arm getInstance() {
        if(instance == null) {instance = new Arm();}
        return instance;
      }
}
