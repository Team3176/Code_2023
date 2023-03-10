// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.intake;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Template hardware interface for a closed loop subsystem. */
public interface IntakeIO{
  /** Contains all of the input data received from hardware. */
  public static class IntakeIOInputs implements LoggableInputs {
    public double velocity = 0.0;
    public double appliedVolts = 0.0;
    public boolean isLinebreak = true;
    public double[] currentAmps = new double[] {};
    public double[] tempCelcius = new double[] {};
    public double PositionFront = 0.0;
    public double PositionBack = 0.0;
    public boolean iscone = false;
    public boolean iscube = false;
    public boolean isextended = false;



    public void toLog(LogTable table) {
      table.put("VelocityOutputPercent", velocity);
      table.put("isLinebreakOne", isLinebreak);
      table.put("AppliedVolts", appliedVolts);
      table.put("CurrentAmps", currentAmps);
      table.put("TempCelcius", tempCelcius);
      table.put("PositionFront", PositionFront);
      table.put("PositionBack", PositionBack);
      table.put("iscone", iscone);
      table.put("iscube", iscube);
      table.put("isextended", isextended);

    }

    public void fromLog(LogTable table) {
      velocity = table.getDouble("VelocityOutputPercent", velocity);
      isLinebreak = table.getBoolean("isLinebreakOne", isLinebreak);
      appliedVolts = table.getDouble("AppliedVolts", appliedVolts);
      currentAmps = table.getDoubleArray("CurrentAmps", currentAmps);
      tempCelcius = table.getDoubleArray("TempCelcius", tempCelcius);
      PositionFront = table.getDouble("PositionFront", PositionFront);
      PositionBack = table.getDouble("PositionBack", PositionBack);
      iscone = table.getBoolean("iscone", iscone);
      iscube = table.getBoolean("iscube", iscube);
      isextended = table.getBoolean("isextended", isextended);
    }
  }



  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /**
   * Run closed loop at the specified velocity.
   * 
   * @param velocityRadPerSec Velocity setpoint.
   */
  public default void setVelocity(double velocityRadPerSec) {}

  /** Retract or Extend pistons */
  public default void setIntakePosition(boolean isextended) {}
}
