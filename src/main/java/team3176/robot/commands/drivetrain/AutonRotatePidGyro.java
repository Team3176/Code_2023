// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.drivetrain.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import team3176.robot.util.God.PID3176;


import javax.sound.sampled.SourceDataLine;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class AutonRotatePidGyro extends CommandBase {
  /** Creates a new TrapezoidDrive. */
  private Drivetrain m_Drivetrain = Drivetrain.getInstance();
  private TrapezoidProfile profile;
  private double botCircumferance = 12.315;  //feet
  private double initial_yaw;
  private double requestedYawChange;
  private double yaw_TrueSetpoint;
  private PIDController rotationController;
  //private PID3176 rotationController;
  private double deadbandOfRequestedYawChange; 

  /** 
   * @param angle change in angle (degrees)
   */
  public AutonRotatePidGyro(double requestedYawChange) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Drivetrain);
    this.requestedYawChange= requestedYawChange;
    rotationController = new PIDController(.15, 0, 0);
    deadbandOfRequestedYawChange = 3;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initial_yaw = m_Drivetrain.getChassisYaw().getRadians();
    this.yaw_TrueSetpoint = initial_yaw + this.requestedYawChange;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double yawError = m_Drivetrain.getChassisYaw().getRadians() - this.yaw_TrueSetpoint; 
    double spinCommand = rotationController.calculate(yawError,this.yaw_TrueSetpoint);
    double smallnum = Math.pow(10,-9);
    m_Drivetrain.drive(smallnum, smallnum, spinCommand);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    double smallnum = Math.pow(10,-10);
    //m_Drivetrain.drive(smallnum, smallnum, smallnum);
    //m_Drivetrain.drive(0,0,0);
    m_Drivetrain.stopMotors();
    //System.out.println("######################################################################################################################     TrapRot.end()");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return profile.isFinished(timer.get());
    if ((m_Drivetrain.getChassisYaw().getRadians() < (this.yaw_TrueSetpoint + this.deadbandOfRequestedYawChange)) && (m_Drivetrain.getChassisYaw().getRadians() > (this.yaw_TrueSetpoint - this.deadbandOfRequestedYawChange))) {
      return true;
    }
    return false;
  }
}
