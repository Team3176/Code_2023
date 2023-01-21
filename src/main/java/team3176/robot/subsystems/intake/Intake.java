// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Intake extends SubsystemBase {
    private TalonSRX motorcontrol = new TalonSRX(22);
  /** Creates a new Intake. */
  public Intake() {
    
    public void spinVelocityPercent(double pct) {
        motorcontrol.set(TalonSRXControlMode.PercentOutput, pct);
       
      }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
