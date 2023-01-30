// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.drivetrain.Drivetrain.coordType;

import java.sql.Time;

import javax.sound.sampled.SourceDataLine;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class TrapezoidRotate extends CommandBase {
  /** Creates a new TrapezoidDrive. */
  private Drivetrain m_Drivetrain = Drivetrain.getInstance();
  private Timer timer;
  private TrapezoidProfile profile;
  private double theta;
  private double distanceX;
  private double distanceY;
  private double rotation_distance, direction;
  private double botCircumferance = 12.315;  //feet
  private double botCircumferancePer1Degree;

  /**
   * 
   * @param direction  +1 makes it turn right, -1 makes it turn left
   * @param angle  scales as follows
   *                5=~10 degrees
   *                10 = 45
   *                20 = 90
   *                22.5 = 135
   *                25 = 180
   *                27.5 = 225
   *                32.5 - 35 = 270
   *                42.5 - 45 = 325
   *                52.5 = 360
   */
  public TrapezoidRotate(double direction, double angle) { //TODO: FIIIIIIIIXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXxx
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Drivetrain);
    //this.distanceX = distanceX;
    //this.distanceY = distanceY;
    this.botCircumferancePer1Degree = botCircumferance / 360;
    //double hyp = Math.sqrt(distanceX * distanceX + distanceY * distanceY);
    //this.theta = Math.atan(distanceY / distanceX);
    this.direction = -direction;
    this.rotation_distance = angle * botCircumferancePer1Degree;
    profile = new TrapezoidProfile
    (new TrapezoidProfile.Constraints(14, 12), new TrapezoidProfile.State(rotation_distance, 0), new TrapezoidProfile.State(0, 0));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Drivetrain.currentCoordType = coordType.ROBOT_CENTRIC;
    //m_gyro.setSpinLockAngle();
    //m_gyro.setSpinLock(true);
    timer = new Timer();
    //timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println("timer for auto: " + timer.get());
    TrapezoidProfile.State setPoint = profile.calculate(timer.get());
    /// double velocityX = setPoint.velocity * Math.cos(theta);
    //double velocityY = setPoint.velocity * Math.sin(theta);
    double spinCommand = setPoint.velocity * rotation_distance;
    if (this.direction > 0) {spinCommand *= -1;}
    double smallnum = Math.pow(10,-9);
    m_Drivetrain.drive(smallnum, smallnum, spinCommand);
    System.out.println("auto trapeziod constants " + timer.get() + " : " + spinCommand);

    // System.out.println("X: " + distanceX + ", Y: " + distanceY);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    double smallnum = Math.pow(10,-10);
    //m_Drivetrain.drive(smallnum, smallnum, smallnum);
    //m_Drivetrain.drive(0,0,0);
    m_Drivetrain.stopMotors();
    timer.stop();
    //System.out.println("######################################################################################################################     TrapRot.end()");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return profile.isFinished(timer.get());
  }
}
