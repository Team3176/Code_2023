// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.drivetrain.Drivetrain.coordType;

public class AutonDrive extends CommandBase {
  /** Creates a new AutonDrive. */

  Timer timer = new Timer();

  private double xVel;
  private double yVel;
  private double omega; //rotational velocity
  private double endTime;
  private Drivetrain drivetrain = Drivetrain.getInstance();

  public AutonDrive(double x, double y, double rot, double time) {
    addRequirements(drivetrain);
    endTime = time;
    xVel = x;
    yVel = y;
    omega = rot;

  }
 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    timer.start();
    drivetrain.currentCoordType = coordType.FIELD_CENTRIC;
    drivetrain.setSpinLockAngle();
    drivetrain.setSpinLock(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(xVel,yVel,omega);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(timer.get() > endTime){
      return true;
    }
    return false;
  }
}
