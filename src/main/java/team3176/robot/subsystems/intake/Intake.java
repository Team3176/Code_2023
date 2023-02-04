// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.TimestampedDouble;
import edu.wpi.first.wpilibj.DigitalInput;
import team3176.robot.subsystems.intake.motorconstants;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import java.security.interfaces.XECPublicKey;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends SubsystemBase {
    private TalonSRX motorcontrol = new TalonSRX(22);
    private I2C.Port m_I2C = I2C.Port.kOnboard;
    private DoubleSolenoid piston;
    private DigitalInput linebreak; 
    private ColorSensorV3 m_ColorSensor = new ColorSensorV3(m_I2C);
    private boolean isInIntake;
    private boolean isCone;
    private boolean isSquircle;
    private boolean isExtended;
    private static Intake instance;
    DoublePublisher dblPub;
    //final DoubleSubscriber dblSub;
    DoubleTopic dblTopic;
    DoublePublisher XPub;
    DoublePublisher YPub;
  /** Creates a new Intake. */
  public Intake(){
    //dblSub = dblTopic.subscribe(0,PubSubOption.keepDuplicates(true), PubSubOption.pollStorage(10));
    //dblPub.setDefault(0.0);
    //dblPub = dblTopic.publish();
    //dblPub = dblTopic.publish(PubSubOption.keepDuplicates(true));
    //dblPub = dblTopic.publishEx("double","{\"myprop\": 5}");
    motorcontrol.configFactoryDefault();
    piston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 4);
    isInIntake = false;
    isCone = false;
    isSquircle = false;
    isExtended = false;
    linebreak = new DigitalInput(0);
    SmartDashboard.setDefaultBoolean("isInIntake", isInIntake);
    SmartDashboard.setDefaultBoolean("isCone", isCone);
    SmartDashboard.setDefaultBoolean("isSquircle", isSquircle);
    //Network tables
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("PubSub");
    XPub = table.getDoubleTopic("X").publish();
    YPub = table.getDoubleTopic("Y").publish();

		
		/* Config the sensor used for Primary PID and sensor direction */
    motorcontrol.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);

		/* Ensure sensor is positive when output is positive */
		motorcontrol.setSensorPhase(true);

    motorcontrol.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0); 
    motorcontrol.configNominalOutputForward(0, motorconstants.kTIMEOUT_MS);
    motorcontrol.configNominalOutputReverse(0, motorconstants.kTIMEOUT_MS);
    motorcontrol.configPeakOutputForward(1.0, motorconstants.kTIMEOUT_MS);
    motorcontrol.configPeakOutputReverse(-1.0, motorconstants.kTIMEOUT_MS);
    
    motorcontrol.configAllowableClosedloopError(motorconstants.kPID_LOOP_IDX, motorconstants.ALLOWABLE_CLOSED_LOOP_ERROR, motorconstants.kTIMEOUT_MS);

    motorcontrol.config_kF(motorconstants.kPID_LOOP_IDX, motorconstants.kF, motorconstants.kTIMEOUT_MS);
    motorcontrol.config_kP(motorconstants.kPID_LOOP_IDX, motorconstants.kP, motorconstants.kTIMEOUT_MS);
    motorcontrol.config_kI(motorconstants.kPID_LOOP_IDX, motorconstants.kI, motorconstants.kTIMEOUT_MS);
    motorcontrol.config_kD(motorconstants.kPID_LOOP_IDX, motorconstants.kD, motorconstants.kTIMEOUT_MS);
    motorcontrol.config_IntegralZone(motorconstants.kPID_LOOP_IDX, motorconstants.kIzone, motorconstants.kTIMEOUT_MS);
    motorcontrol.setInverted(true);
    
    motorcontrol.configNominalOutputReverse(0, motorconstants.kTIMEOUT_MS);
    motorcontrol.configPeakOutputReverse(-1, motorconstants.kTIMEOUT_MS);
    motorcontrol.configPeakOutputForward(1, motorconstants.kTIMEOUT_MS);
    motorcontrol.configNominalOutputForward(0, motorconstants.kTIMEOUT_MS);
  }

  public void spinVelocityPercent(double pct) {
    motorcontrol.set(TalonSRXControlMode.PercentOutput, pct);
   
  }
  
  public void setPosition(int i) {
    double currentPos = motorcontrol.getSelectedSensorPosition(0);
    motorcontrol.set(ControlMode.Position, currentPos + i);
  }

  public void Extend() {
    piston.set(Value.kForward);
    isExtended = true;
  }

  public void Retract() {
    piston.set(Value.kReverse);
    isExtended = false;
  }

  public static Intake getInstance(){
    if ( instance == null ) {
      instance = new Intake();
    }
    return instance;
  }

  public Boolean getLinebreak()
  {
    return linebreak.get();
  }

  //Cool example of a subsystem holding basic commands
  public Command stopIntakeCommand() {
    return this.runOnce(() -> this.spinVelocityPercent(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Color detectedColor = m_ColorSensor.getColor();
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    XPub.set(1.0);
    YPub.set(2.0, 0);
    long time = NetworkTablesJNI.now();
    //dblPub.set(3.0, time);
    //myFunc(dblPub);
    //dblPub.close();
    //double val = dblSub.get();
    //double val = dblSub.get(-1.0);
    //double val = dblSub.getAsDouble();
    //TimestampedDouble tsVal = dblSub.getAtomic();
    //TimestampedDouble[] tsUpdates = dblSub.readQueue();
    //double[] valUpdates = dblSub.readQueueValues();

  

    if ((0.35 <= detectedColor.red && detectedColor.red <= 0.379) && 
        (0.466 <= detectedColor.green && detectedColor.green <= 0.516) && 
        (0.083 <= detectedColor.blue && detectedColor.blue <= 0.188))
    {
      isCone = true;
      // System.out.println("TRUE");
    }
    else 
    {
      isCone = false;
      // System.out.println("FALSE");
    }
    SmartDashboard.putBoolean("isCone", isCone);
    
    if ((0.241 <= detectedColor.red && detectedColor.red <= 0.318) && 
        (0.382 <= detectedColor.green && detectedColor.green <= 0.458) && 
        (0.227 <= detectedColor.blue && detectedColor.blue <= 0.375))
    {
      isSquircle = true;
    }
    else 
    {
      isSquircle = false;
    }
    SmartDashboard.putBoolean("isSquircle", isSquircle);



    // Code stating if something is in the Intake
    if (m_ColorSensor.getProximity() <= 1800)
    {
      if (linebreak.get() == false)
      {
        isInIntake = true;
        setPosition(1000);
      }
      else
      {
        isInIntake = false;
      }
    }
    SmartDashboard.putBoolean("isInIntake", isInIntake);
   }

 
}


