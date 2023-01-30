// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.drivetrain.Drivetrain.coordType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class TrapezoidDriveRotate extends CommandBase {
  /** Creates a new TrapezoidDrive. */
  private Drivetrain m_Drivetrain = Drivetrain.getInstance();

  private Timer timer = new Timer();
  private TrapezoidProfile driveProfile, spinProfile;
  private double theta;
  private double distanceX;
  private double distanceY;
  private double rotation_distance, direction;
  private double botCircumferance = 12.315;  //feet
  private double botCircumferancePer1Degree;

  public TrapezoidDriveRotate(double distanceX, double distanceY, double spinDirection, double angle) { //TODO: FIIIIIIIIXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXxx
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Drivetrain);
    this.distanceX = distanceX;
    this.distanceY = distanceY;
    double hyp = Math.sqrt(distanceX * distanceX + distanceY * distanceY);
    this.theta = Math.atan(distanceY / distanceX);
    this.botCircumferancePer1Degree = botCircumferance / 360;
    this.direction = direction;
    this.rotation_distance = angle * botCircumferancePer1Degree;

    driveProfile = new TrapezoidProfile
    (new TrapezoidProfile.Constraints(14, 12), new TrapezoidProfile.State(hyp, 0), new TrapezoidProfile.State(0, 0));
    spinProfile = new TrapezoidProfile
    (new TrapezoidProfile.Constraints(14, 12), new TrapezoidProfile.State(rotation_distance, 0), new TrapezoidProfile.State(0, 0));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Drivetrain.currentCoordType = coordType.ROBOT_CENTRIC;
    //m_gyro.setSpinLockAngle();
    //m_gyro.setSpinLock(true);
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    TrapezoidProfile.State driveSetPoint = driveProfile.calculate(timer.get());
    double velocityX = driveSetPoint.velocity * Math.cos(theta);
    double velocityY = driveSetPoint.velocity * Math.sin(theta);
    TrapezoidProfile.State spinSetPoint = spinProfile.calculate(timer.get());
    double spinCommand = spinSetPoint.velocity * rotation_distance;
    if (this.direction > 0) {spinCommand *= -1;}
    m_Drivetrain.drive(Math.copySign(velocityX, distanceX), Math.copySign(velocityY, distanceY), spinCommand);

    // System.out.println("X: " + distanceX + ", Y: " + distanceY);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //drivetrain.setSpinLock(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (driveProfile.isFinished(timer.get()) && (spinProfile.isFinished(timer.get())));
  }
}
