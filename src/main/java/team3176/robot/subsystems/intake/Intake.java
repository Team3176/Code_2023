// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


//import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class Intake extends SubsystemBase {
    private TalonSRX motorcontrol = new TalonSRX(22);
    public static final int kTimeoutMs = 0;
    public static final int kPID_LOOP_IDX = 0;
    public static final double ALLOWABLE_CLOSED_LOOP_ERROR = 0;
  /** Creates a new Intake. */
  public Intake() {

    motorcontrol.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0); 
    this.motorcontrol.configNominalOutputForward(0, motorcontrol.kTIMEOUT_MS);
    this.motorcontrol.configNominalOutputReverse(0, motorcontrol.kTIMEOUT_MS);
    this.motorcontrol.configPeakOutputForward(1.0, motorcontrol.kTIMEOUT_MS);
    this.motorcontrol.configPeakOutputReverse(-1.0, motorcontrol.kTIMEOUT_MS);
    
    this.motorcontrol.configAllowableClosedloopError(motorcontrol.kPID_LOOP_IDX, motorcontrol.ALLOWABLE_CLOSED_LOOP_ERROR, motorcontrol.kTIMEOUT_MS);

    this.motorcontrol.config_kF(motorcontrol.kPID_LOOP_IDX, motorcontrol.PIDFConstants[0][3], motorcontrol.kTIMEOUT_MS);
    this.motorcontrol.config_kP(motorcontrol.kPID_LOOP_IDX, motorcontrol.PIDFConstants[0][0], motorcontrol.kTIMEOUT_MS);
    this.motorcontrol.config_kI(motorcontrol.kPID_LOOP_IDX, motorcontrol.PIDFConstants[0][1], motorcontrol.kTIMEOUT_MS);
    this.motorcontrol.config_kD(motorcontrol.kPID_LOOP_IDX, motorcontrol.PIDFConstants[0][2], motorcontrol.kTIMEOUT_MS);
    this.motorcontrol.config_IntegralZone(motorcontrol.kPID_LOOP_IDX, motorcontrol.PIDFConstants[0][4], motorcontrol.kTIMEOUT_MS);
    this.motorcontrol.setInverted(true);
  }

  public void spinVelocityPercent(double pct) {
    motorcontrol.set(TalonSRXControlMode.PercentOutput, pct);
   
  }
 
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}


