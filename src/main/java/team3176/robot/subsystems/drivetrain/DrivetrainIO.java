// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Template hardware interface for a closed loop subsystem. */
public interface DrivetrainIO{
  /** Contains all of the input data received from hardware. */
  public static class DrivetrainIOInputs implements LoggableInputs {
    public double position_1 = 0.0;
    public double position_2 = 0.0;
    public double position_3 = 0.0;
    public double position_4 = 0.0;
    public double velocity_1 = 0.0;
    public double velocity_2 = 0.0;
    public double velocity_3 = 0.0;
    public double velocity_4 = 0.0;
    public double appliedVolts_1 = 0.0;
    public double appliedVolts_2 = 0.0;
    public double appliedVolts_3 = 0.0;
    public double appliedVolts_4 = 0.0;
    public double appliedVolts_5 = 0.0;
    public double appliedVolts_6 = 0.0;
    public double appliedVolts_7 = 0.0;
    public double appliedVolts_8 = 0.0;
    public double[] currentAmps_1 = new double[] {};
    public double[] currentAmps_2 = new double[] {};
    public double[] currentAmps_3 = new double[] {};
    public double[] currentAmps_4 = new double[] {};
    public double[] currentAmps_5 = new double[] {};
    public double[] currentAmps_6 = new double[] {};
    public double[] currentAmps_7 = new double[] {};
    public double[] currentAmps_8 = new double[] {};
    public double[] tempCelcius_1 = new double[] {};
    public double[] tempCelcius_2 = new double[] {};
    public double[] tempCelcius_3 = new double[] {};
    public double[] tempCelcius_4 = new double[] {};
    public double[] tempCelcius_5 = new double[] {};
    public double[] tempCelcius_6 = new double[] {};
    public double[] tempCelcius_7 = new double[] {};
    public double[] tempCelcius_8 = new double[] {};

    public void toLog(LogTable table) {
      table.put("Position 1", position_1);
      table.put("Position 2", position_2);
      table.put("Position 3", position_3);
      table.put("Position 4", position_4);
      table.put("Velocity 1", velocity_1);
      table.put("Velocity 2", velocity_2);
      table.put("Velocity 3", velocity_3);
      table.put("Velocity 4", velocity_4);
      table.put("AppliedVolts 1", appliedVolts_1);
      table.put("AppliedVolts 2", appliedVolts_2);
      table.put("AppliedVolts 3", appliedVolts_3);
      table.put("AppliedVolts 4", appliedVolts_4);
      table.put("AppliedVolts 5", appliedVolts_5);
      table.put("AppliedVolts 6", appliedVolts_6);
      table.put("AppliedVolts 7", appliedVolts_7);
      table.put("AppliedVolts 8", appliedVolts_8);
      table.put("CurrentAmps 1", currentAmps_1);
      table.put("CurrentAmps 2", currentAmps_2);
      table.put("CurrentAmps 3", currentAmps_3);
      table.put("CurrentAmps 4", currentAmps_4);
      table.put("CurrentAmps 5", currentAmps_5);
      table.put("CurrentAmps 6", currentAmps_6);
      table.put("CurrentAmps 7", currentAmps_7);
      table.put("CurrentAmps 8", currentAmps_8);
      table.put("TempCelcius 1", tempCelcius_1);
      table.put("TempCelcius 2", tempCelcius_2);
      table.put("TempCelcius 3", tempCelcius_3);
      table.put("TempCelcius 4", tempCelcius_4);
      table.put("TempCelcius 5", tempCelcius_5);
      table.put("TempCelcius 6", tempCelcius_6);
      table.put("TempCelcius 7", tempCelcius_7);
      table.put("TempCelcius 8", tempCelcius_8);
    }

    public void fromLog(LogTable table) {
      position_1 = table.getDouble("Position 1", position_1);
      position_2 = table.getDouble("Position 2", position_2);
      position_3 = table.getDouble("Position 3", position_3);
      position_4 = table.getDouble("Position 4", position_4);
      velocity_1 = table.getDouble("Velocity 1", velocity_1);
      velocity_2 = table.getDouble("Velocity 2", velocity_2);
      velocity_3 = table.getDouble("Velocity 3", velocity_3);
      velocity_4 = table.getDouble("Velocity 4", velocity_4);
      appliedVolts_1 = table.getDouble("AppliedVolts 1", appliedVolts_1);
      appliedVolts_2 = table.getDouble("AppliedVolts 2", appliedVolts_2);
      appliedVolts_3 = table.getDouble("AppliedVolts 3", appliedVolts_3);
      appliedVolts_4 = table.getDouble("AppliedVolts 4", appliedVolts_4);
      appliedVolts_5 = table.getDouble("AppliedVolts 5", appliedVolts_5);
      appliedVolts_6 = table.getDouble("AppliedVolts 6", appliedVolts_6);
      appliedVolts_7 = table.getDouble("AppliedVolts 7", appliedVolts_7);
      appliedVolts_8 = table.getDouble("AppliedVolts 8", appliedVolts_8);
      currentAmps_1 = table.getDoubleArray("CurrentAmps 1", currentAmps_1);
      currentAmps_2 = table.getDoubleArray("CurrentAmps 2", currentAmps_2);
      currentAmps_3 = table.getDoubleArray("CurrentAmps 3", currentAmps_3);
      currentAmps_4 = table.getDoubleArray("CurrentAmps 4", currentAmps_4);
      currentAmps_5 = table.getDoubleArray("CurrentAmps 5", currentAmps_5);
      currentAmps_6 = table.getDoubleArray("CurrentAmps 6", currentAmps_6);
      currentAmps_7 = table.getDoubleArray("CurrentAmps 7", currentAmps_7);
      currentAmps_8 = table.getDoubleArray("CurrentAmps 8", currentAmps_8);
      tempCelcius_1 = table.getDoubleArray("TempCelcius 1", tempCelcius_1);
      tempCelcius_2 = table.getDoubleArray("TempCelcius 2", tempCelcius_2);
      tempCelcius_3 = table.getDoubleArray("TempCelcius 3", tempCelcius_3);
      tempCelcius_4 = table.getDoubleArray("TempCelcius 4", tempCelcius_4);
      tempCelcius_5 = table.getDoubleArray("TempCelcius 5", tempCelcius_5);
      tempCelcius_6 = table.getDoubleArray("TempCelcius 6", tempCelcius_6);
      tempCelcius_7 = table.getDoubleArray("TempCelcius 7", tempCelcius_7);
      tempCelcius_8 = table.getDoubleArray("TempCelcius 8", tempCelcius_8);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(DrivetrainIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Encoder Position of the Drivetrain */
  public default void setDrivetrainPosition(double position) {}

  public default void setDrivetrainVelocity(double velocity) {}
}
