// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot;

import team3176.robot.constants.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.util.HashMap;
import java.util.Map;
//import team3176.robot.subsystems.*;
//import team3176.robot.subsystems.arm.*;
import team3176.robot.subsystems.claw.*;
import team3176.robot.subsystems.controller.*;
//import team3176.robot.subsystems.drivetrain.*;
//import team3176.robot.subsystems.drivetrain.CoordSys.coordType;
import team3176.robot.subsystems.intake.*;
import team3176.robot.subsystems.signalling.*;
import team3176.robot.subsystems.vision.*;
import team3176.robot.subsystems.intake.Intake;
import team3176.robot.subsystems.vision.Vision;
//import team3176.robot.commands.arm.*;
//import team3176.robot.commands.autons.*;
//import team3176.robot.commands.claw.*;
//import team3176.robot.commands.drivetrain.*;
//import team3176.robot.commands.intake.*;
//import team3176.robot.commands.signalling.*;
//import team3176.robot.commands.vision.*;
//import team3176.robot.commands.test.*;
//import team3176.robot.commands.util.*;

public class RobotContainer {

  private final PowerDistribution m_PDH;
  private final Compressor m_Compressor;
  //private final SwerveSubsystem m_SwerveSubsystem;
  //private final CoordSys m_CoordSys;
  //private final Arm m_Arm;
  private final Claw m_Claw;
  private final Controller m_Controller;
  //private final Drivetrain m_Drivetrain;
  private final Intake m_Intake;
  private final Signalling m_Signalling;
  private final Vision m_Vision;

  private SendableChooser<String> m_autonChooser;
  // private static final String m_B = "s_Block";
  private static final String m_M = "s_ExitTarmac";
  private static final String M_EXITANDTUR_STRING = "s_ExitAndTurn";
  private static final String m_6L = "s_Move6inToTheLeft";
  private static final String m_6R = "s_Move6inToTheRight";
  private static final String m_6F = "s_Move6inToTheFront";
  private static final String m_6B = "s_Move6inToTheBack";
  private static final String m_9F = "s_Move9inToTheFront";
  private static final String m_9B = "s_Move9inToTheBack";
  private static final String m_TS = "s_ShootAndLeave";
  private static final String m_SI = "s_LeaveAndShootTwo";
  private static final String m_2H = "s_2BallHanger";
  private static final String m_2M = "s_2BallMid";
  private static final String m_MS = "s_MoveAndShoot";
  private static final String m_3B = "s_3Ball";
  private static final String m_3BS = "s_3BallSlow";
  private static final String m_4B = "s_4Ball";
  private static final String m_4G = "s_4BallGyro";
  private static final String m_5B = "s_5Ball";
  private static final String m_3H = "s_3BallHanger";
  private static final String m_2C = "s_2BallCitrus";
  private static final String m_2EC = "s_2BallExtraCitrus";
  private static final String m_Int = "s_Interfere";
  private static final String m_Rot = "s_Rot";
  private static final String m_TrapRot = "s_TrapRot";
  private static final String m_TrapDriveRot = "s_TrapDriveRot";

  public RobotContainer() {
    //m_Arm = Arm.getInstance();
    m_Claw = Claw.getInstance();
    m_Controller = Controller.getInstance();
    //m_Drivetrain= Drivetrain.getInstance();
    m_Intake = Intake.getInstance();
    m_Signalling = Signalling.getInstance();
    m_Vision = Vision.getInstance();

    m_PDH = new PowerDistribution(1, ModuleType.kRev);
    m_PDH.clearStickyFaults();

    m_Compressor = new Compressor(1, PneumaticsModuleType.REVPH);
    // TODO: ADD A WAY TO CLEAR STICKY FAULTS
    // m_Compressor.disable(); //HAVE TO TELL IT TO DISABLE FOR IT TO NOT AUTO START
    m_Compressor.enableDigital();

    /* 
    if (!LoggerConstants.IS_TUNING_MODE) {
      m_SwerveSubsystem.setDefaultCommand(new SwerveDrive(
          () -> m_Controller.getForward(),
          () -> m_Controller.getStrafe(),
          () -> m_Controller.getSpin()// ,
      ));
    } else {
      m_SwerveSubsystem.setDefaultCommand(new SwerveDriveTune());
    }
    */

    m_autonChooser = new SendableChooser<>();
    m_autonChooser.setDefaultOption("Auto: ExitTarmac", m_M);
    // m_autonChooser.addOption("Auto: Block", m_B);
    m_autonChooser.addOption("Auto: Move 6in Left", m_6L);
    m_autonChooser.addOption("Auto: ExitAndTurn", M_EXITANDTUR_STRING);
    m_autonChooser.addOption("Auto: Move 6in Right", m_6R);
    m_autonChooser.addOption("Auto: Move 6in Forward", m_6F);
    m_autonChooser.addOption("Auto: Move 6in Backwards", m_6B);
    m_autonChooser.addOption("Auto: Move 9in Forward", m_9F);
    m_autonChooser.addOption("Auto: Move 9in Backwards", m_9B);
    m_autonChooser.addOption("Auto: Shoot and Exit Tarmac", m_TS);
    m_autonChooser.addOption("Auto: 2 Ball (Right)", m_SI);
    m_autonChooser.addOption("Auto: 2 Ball (Left/Hanger)", m_2H);
    m_autonChooser.addOption("Auto: 2 Ball (Middle)", m_2M);
    m_autonChooser.addOption("Auto: Exit and Shoot", m_MS);
    m_autonChooser.addOption("Auto: 3 Ball (Right)", m_3B);
    m_autonChooser.addOption("Auto: 3 Ball Slow (Right)", m_3BS);
    m_autonChooser.addOption("Auto: 3 Ball (Left/Hanger)", m_3H);
    m_autonChooser.addOption("Auto: 4 Ball", m_4B);
    m_autonChooser.addOption("Auto: 4 Ball Gyro", m_4G);
    m_autonChooser.addOption("Auto: 5 Ball", m_5B);
    m_autonChooser.addOption("Auto: 2 Ball Citrus (Left/Hanger)", m_2C);
    m_autonChooser.addOption("Auto: 2 Ball Extra Citrus (Left/Hanger)", m_2EC);
    m_autonChooser.addOption("Auto: Interfere (Left/Hanger)", m_Int);
    m_autonChooser.addOption("Auto: Rotation", m_Rot);
    m_autonChooser.addOption("Auto: TrapRotate", m_TrapRot);
    m_autonChooser.addOption("Auto: TrapDriveRotate", m_TrapDriveRot);
    SmartDashboard.putData("Auton Choice", m_autonChooser);

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    //m_Controller.getTransStick_Button1().whileTrue(new InstantCommand( () -> m_Drivetrain.setTurbo(true), m_SwerveSubsystem));
    //m_Controller.getTransStick_Button1().onFalse(new InstantCommand( () -> m_Drivetrain.setTurbo(false), m_SwerveSubsystem));
    //m_Controller.getTransStick_Button3().whileTrue(new SwerveDefense());
    // m_Controller.getTransStick_Button4().whenPressed(new ToggleCoordSys());
    //m_Controller.getTransStick_Button4().whileTrue(new InstantCommand(m_CoordSys::setCoordTypeToRobotCentric,m_CoordSys));
    //m_Controller.getTransStick_Button4().onFalse(new InstantCommand(m_CoordSys::setCoordTypeToFieldCentric,m_CoordSys));

    //m_Controller.getRotStick_Button1().whileTrue(new FlywheelAngleVisionIntAutoFire());
    //m_Controller.getRotStick_Button1().whileTrue(new VisionSpinCorrectionOn());
    //m_Controller.getRotStick_Button1().onFalse(new VisionSpinCorrectionOff());
    //m_Controller.getRotStick_Button2().whileTrue(new ShootVisionAutoFire());
    
    //m_Controller.getRotStick_Button3().whileTrue(new IntakingDirect2());
    //m_Controller.getRotStick_Button3().onFalse(new DelayedIntakeStop());
    //m_Controller.getRotStick_Button3().whenReleased(new SwerveSpinLockOff());
    //m_Controller.getRotStick_Button3().whenReleased(new SwerveSpinLockOff());
    //m_Controller.getRotStick_Button4().onTrue(new SwerveResetGyro());

    // m_Controller.getRotStick_Button5().whenPressed(new
    // SwervePodsAzimuthGoHome());

    //m_Controller.operator.a().onTrue(new IntakingDirect2());
    //m_Controller.operator.a().onFalse(new DelayedIntakeStop());

    //m_Controller.operator.y().whileTrue(new ShootSetVals());
    //m_Controller.operator.b().onTrue(new FlywheelStop());

    //m_Controller.operator.back().and(m_Controller.operator.leftBumper()).and(m_Controller.operator.rightBumper()).onTrue(new IndexerBackWhenHeld());
    //m_Controller.operator.start().and(m_Controller.operator.leftBumper()).onTrue(new IndexerForwardWhenHeld());
    //m_Controller.operator.back().and(m_Controller.operator.leftBumper()).and(m_Controller.operator.rightBumper()).onTrue(new ExtendIntake());
    //m_Controller.operator.start().and(m_Controller.operator.leftBumper()).and(m_Controller.operator.rightBumper()).onTrue(new RetractIntake());

    // m_Controller.getOp_A_DS().onTrue(new ClimbPistonEngage()); // TODO: CHECK IF TWO COMMANDS CAN BE MAPPED TO THE
    //                                                                // SAME BUTTON
    // m_Controller.getOp_A_DS().whenActive(new AnglerZeroAtMax());
    //m_Controller.operator.b().and(m_Controller.operator.leftBumper()).and(m_Controller.operator.rightBumper()).onTrue(new ClimbPistonRetract());
    //m_Controller.operator.b().and(m_Controller.operator.leftBumper()).and(m_Controller.operator.rightBumper()).onTrue(new AnglerZeroAtMax());
    // m_Controller.getOp_X_DS().whenActive(new ClimbPistonEngage());
    //m_Controller.operator.a().and(m_Controller.operator.leftBumper()).and(m_Controller.operator.rightBumper()).onTrue(new ClimbPistonEngage());
    //m_Controller.operator.y().and(m_Controller.operator.leftBumper()).and(m_Controller.operator.rightBumper()).onTrue(new AnglerZeroAtMax());

    //m_Controller.operator.povUp().onTrue(new VisionDriverCam());
    //m_Controller.operator.povDown().onTrue(new VisionZoom2x());

    //m_Controller.operator.povLeft().onTrue(new FlywheelAngleFender());
    //m_Controller.operator.povRight().onTrue(new FlywheelAngleWall());

    //m_Controller.operator.a().and(m_Controller.operator.leftBumper()).whileTrue(new AnglerZeroAtMax());
    //m_Controller.operator.y().and(m_Controller.operator.leftBumper()).onTrue(new FlywheelDefaultCommandStop());

    //m_Controller.operator.back().whileTrue(new SpittingDown());
    //m_Controller.operator.start().whileTrue(new SpittingUp());

    // m_Controller.getOp_Back().whileTrue(new FlywheelPIDToggleTest());
    // m_Controller.getOp_Start().whileTrue(new ShootPIDToggleTest());

    //m_Controller.operator.x().whileTrue(new FlywheelAngleVision());

    //m_Controller.operator.leftBumper().whileTrue(new ShootVision());

    //m_Controller.operator.rightBumper().whileTrue(new FlywheelAngleVisionInt());
  }

  /* 
  public void AutonInitRobotCentric() {
    m_CoordSys.setCoordTypeToRobotCentric();
  }

  public void TelopInitFieldCentric() {
    m_CoordSys.setCoordTypeToFieldCentric();
  }
  */

  /* 
  public Command getAutonomousCommand() {
    String chosen = m_autonChooser.getSelected();

    if (chosen.equals(m_M))
      return new AutonExitTarmac();
    // if(chosen.equals(m_B)) return new AutonBlock();
    if (chosen.equals(M_EXITANDTUR_STRING))
      return new  ExitAndTurn();
    if (chosen.equals(m_6L))
      return new TrapezoidDrive(0, 6);
    if (chosen.equals(m_6R))
      return new TrapezoidDrive(0, -6);
    if (chosen.equals(m_6F))
      return new TrapezoidDrive(6, 0);
    if (chosen.equals(m_6B))
      return new TrapezoidDrive(-6, 0);
    if (chosen.equals(m_9F))
      return new TrapezoidDrive(9, 0);
    if (chosen.equals(m_9B))
      return new TrapezoidDrive(-9, 0);
    if (chosen.equals(m_TS))
      return new AutoInTarmacShoot();
    if (chosen.equals(m_SI))
      return new Auto2Balls();
    if (chosen.equals(m_2H))
      return new Auto2BallsAtHanger();
    if (chosen.equals(m_2M))
      return new Auto2BallsMiddle();
    if (chosen.equals(m_MS))
      return new AutoMoveAndShoot();
    if (chosen.equals(m_3B))
      return new Auto3Balls();
    if (chosen.equals(m_3BS))
      return new Auto3BallSlow();
    if (chosen.equals(m_4B))
      return new Auto4Ball();
    if (chosen.equals(m_4G))
      return new Auto4BallGyro();
    if (chosen.equals(m_5B))
      return new Auto5Ball();
    if (chosen.equals(m_3H))
      return new Auton3BallAtHanger();
    if (chosen.equals(m_2C))
      return new Auto2BallCitrus();
    if (chosen.equals(m_2EC))
      return new Auto2BallExtraCitrus();
    if (chosen.equals(m_Int))
      return new AutoInterfere();
    if (chosen.equals(m_Rot))
      return new AutonRotate(-0.15, 90);
    if (chosen.equals(m_TrapRot))
      return new TrapezoidRotate(-1, 20);
    if (chosen.equals(m_TrapDriveRot))
      return new TrapezoidDriveRotate(3, 0, 1, 5);

    return new AutoInTarmacShoot();

    //return Commands.print("No autonomous command configured");
  }
  */

  private static class AutonRoutine {
    public final AutonStartPosition startPosition;
    public final Command command;

    public AutonRoutine(AutonStartPosition startPosition, Command command) {
      this.startPosition = startPosition;
      this.command = command;
    }
  }

  public static enum AutonStartPosition {
    TARMAC_RIGHT, TARMAC_MIDDLE, TARMAC_LEFT;

    public Pose2d getPose() {
      switch (this) {
        case TARMAC_RIGHT:
          return new Pose2d();
        case TARMAC_MIDDLE:
          return new Pose2d();
        case TARMAC_LEFT:
          return new Pose2d();
        default:
          return new Pose2d();
      } 
    }
  }
  
}
