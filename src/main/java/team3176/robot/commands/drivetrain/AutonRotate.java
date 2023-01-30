// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.drivetrain;

import java.sql.Driver;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.drivetrain.Drivetrain.coordType;

public class AutonRotate extends CommandBase {

  private Drivetrain drivetrain = Drivetrain.getInstance();
  private double rotationSpeed;
  private double degrees;
  private double initialAngle;
  private double goal;
  private double currentAngle;

  /** To go in the negative direction put a negative rotational speed and positive degrees. */
  public AutonRotate(double rot, double degrees) {
    addRequirements(drivetrain);
    rotationSpeed = rot;
    this.degrees = degrees;
  }

  @Override
  public void initialize() {
    drivetrain.currentCoordType = coordType.ROBOT_CENTRIC;
    //initialAngle = -drivetrain.getNavxAngle_inDegrees();
    initialAngle = drivetrain.getChassisYaw().getDegrees();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(0,0,rotationSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //TODO: rotataion 2D rewrite

    // if(rotationSpeed > 0){
    //   if(m_gyro.getNavxAngle_inDegrees() >= initialAngle + degrees ){
    //     drivetrain.drive(0,0,0);
    //     return true;
    //   }
    // }
    // if(rotationSpeed < 0){
    //   if(m_gyro.getNavxAngle_inDegrees() <= initialAngle + -degrees ){
    //     drivetrain.drive(0,0,0);
    //     return true;
    //   }
    // }
    return false;
  }
}
