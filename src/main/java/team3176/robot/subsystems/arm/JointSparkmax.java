package team3176.robot.subsystems.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class JointSparkmax
{
    private static final int deviceID = 6;
    private CANSparkMax m_motor;
    private SparkMaxPIDController m_pidController;
    private RelativeEncoder m_encoder;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;



    public JointSparkmax(int deviceID) 
    {
        
        m_motor = new CANSparkMax(deviceID, MotorType.kBrushless);  

        m_motor.restoreFactoryDefaults();

        m_pidController = m_motor.getPIDController();
        m_encoder = m_motor.getEncoder();


        kP = 0.1; 
        kI = 1e-4;
        kD = 1; 
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = 1; 
        kMinOutput = -1;

        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);

        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Zone", kIz);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Max Output", kMaxOutput);
        SmartDashboard.putNumber("Min Output", kMinOutput);
        SmartDashboard.putNumber("Set Rotations", 0);
    }

    
    public void updatePID()
    {

        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);

        if((p != kP)) { m_pidController.setP(p); kP = p; }
        if((i != kI)) { m_pidController.setI(i); kI = i; }
        if((d != kD)) { m_pidController.setD(d); kD = d; }
        if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
        if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
    }   
      
    public void motorMovePercentOutput(){
        m_motor.set(3);

    }






    public void motorMove()
    {
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);
        double rotations = SmartDashboard.getNumber("Set Rotations", 0);
        ///System.out.print("motorMove:outside Conditional");
        //if((max != kMaxOutput) || (min != kMinOutput)) 
        //{ 
            System.out.print("motorMove:conditional");
            m_pidController.setOutputRange(min, max); 
            kMinOutput = min; kMaxOutput = max; 
          
            m_pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
    
            SmartDashboard.putNumber("SetPoint", rotations);
            SmartDashboard.putNumber("ProcessVariable", m_encoder.getPosition());
   
        //}
    }
}  