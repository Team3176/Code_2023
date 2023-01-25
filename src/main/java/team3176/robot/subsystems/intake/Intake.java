// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.intake;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


//import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.fasterxml.jackson.databind.JsonSerializable.Base;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class Intake extends SubsystemBase {
    private TalonSRX motorcontrol = new TalonSRX(22);
  /** Creates a new Intake. */
  public Intake() {

    motorcontrol.configFactoryDefault();
		
		/* Config the sensor used for Primary PID and sensor direction */
    motorcontrol.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);

		/* Ensure sensor is positive when output is positive */
		motorcontrol.setSensorPhase(true);

    // motorcontrol.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0); 
    // motorcontrol.configNominalOutputForward(0, motorconstants.kTIMEOUT_MS);
    // motorcontrol.configNominalOutputReverse(0, motorconstants.kTIMEOUT_MS);
    // motorcontrol.configPeakOutputForward(1.0, motorconstants.kTIMEOUT_MS);
    // motorcontrol.configPeakOutputReverse(-1.0, motorconstants.kTIMEOUT_MS);
    
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
  
  public void set(int i) {
    double currentPos = motorcontrol.getSelectedSensorPosition(0);
    motorcontrol.set(ControlMode.Position, currentPos + i);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

 
}


