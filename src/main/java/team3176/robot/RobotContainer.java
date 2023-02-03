// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot;

import team3176.robot.commands.pipeSwitch;
import team3176.robot.commands.switchLED;
import team3176.robot.constants.*;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import team3176.robot.commands.drivetrain.*;
import team3176.robot.commands.drivetrain.AutonRotate;
import team3176.robot.commands.drivetrain.SwerveDefense;
import team3176.robot.commands.drivetrain.SwerveDrive;
import team3176.robot.commands.drivetrain.SwerveDriveTune;
import team3176.robot.commands.drivetrain.SwerveResetGyro;
import team3176.robot.commands.drivetrain.TrapezoidDrive;
import team3176.robot.commands.drivetrain.TrapezoidDriveRotate;
import team3176.robot.commands.drivetrain.TrapezoidRotate;
//import team3176.robot.commands.util.*;
import team3176.robot.constants.LoggerConstants;
//import team3176.robot.subsystems.*;
//import team3176.robot.subsystems.arm.*;
//import team3176.robot.subsystems.claw.*;
import team3176.robot.subsystems.controller.*;
import team3176.robot.subsystems.drivetrain.*;
import team3176.robot.subsystems.drivetrain.Drivetrain.coordType;
//import team3176.robot.subsystems.drivetrain.CoordSys.coordType;
//import team3176.robot.subsystems.intake.*;
//import team3176.robot.subsystems.signalling.*;
import team3176.robot.subsystems.vision.*;
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
import team3176.robot.subsystems.vision.Vision.LEDState;

public class RobotContainer {

  private final PowerDistribution m_PDH;
  //private final Compressor m_Compressor;
  private final Drivetrain m_Drivetrain;
  //private final CoordSys m_CoordSys;
  //private final Arm m_Arm;
  private final Controller m_Controller;
  private final Vision m_Vision;

  private SendableChooser<String> m_autonChooser;
  // private static final String m_B = "s_Block";
  private static final String m_M = "s_ExitTarmac";

  public RobotContainer() {
    m_Controller = Controller.getInstance();
    m_Drivetrain = Drivetrain.getInstance();
    //m_Arm = Arm.getInstance();
    m_Vision = Vision.getInstance();

    m_PDH = new PowerDistribution(1, ModuleType.kRev);
    m_PDH.clearStickyFaults();

    //m_Compressor = new Compressor(1, PneumaticsModuleType.REVPH);
    // TODO: ADD A WAY TO CLEAR STICKY FAULTS
    // m_Compressor.disable(); //HAVE TO TELL IT TO DISABLE FOR IT TO NOT AUTO START
    //m_Compressor.enableDigital();


    if (!LoggerConstants.IS_TUNING_MODE) {
      m_Drivetrain.setDefaultCommand(new SwerveDrive(
          () -> m_Controller.getForward(),
          () -> m_Controller.getStrafe(),
          () -> m_Controller.getSpin()// ,
      // () -> m_Controller.isFieldCentricButtonPressed(),
      // () -> m_Controller.isRobotCentricButtonPressed()
      ));
    } else {
      m_Drivetrain.setDefaultCommand(new SwerveDriveTune());
    }

    m_autonChooser = new SendableChooser<>();
    m_autonChooser.setDefaultOption("Auto: ExitTarmac", m_M);
    SmartDashboard.putData("Auton Choice", m_autonChooser);

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    m_Controller.getTransStick_Button1().whileTrue(new InstantCommand( () -> m_Drivetrain.setTurbo(true), m_Drivetrain));
    m_Controller.getTransStick_Button1().onFalse(new InstantCommand( () -> m_Drivetrain.setTurbo(false), m_Drivetrain));
    m_Controller.getTransStick_Button3().whileTrue(new SwerveDefense());
    m_Controller.getTransStick_Button4().whileTrue(new InstantCommand( () -> m_Drivetrain.setCoordType(coordType.ROBOT_CENTRIC), m_Drivetrain));
    m_Controller.getTransStick_Button4().onFalse(new InstantCommand( () -> m_Drivetrain.setCoordType(coordType.FIELD_CENTRIC), m_Drivetrain));

    m_Controller.operator.a().onTrue(new switchLED());
    m_Controller.operator.b().onTrue(new pipeSwitch(0));
    m_Controller.operator.y().onTrue(new pipeSwitch(1));
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


  public Command getAutonomousCommand() {
    String chosen = m_autonChooser.getSelected();

    

    return new AutonRotate(-0.15, 90);
  }

  
}
