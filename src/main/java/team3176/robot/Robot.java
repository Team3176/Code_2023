// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.littletonrobotics.junction.inputs.*;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;


/* 
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedNetworkTables;
import org.littletonrobotics.junction.inputs.LoggedSystemStats;
import org.littletonrobotics.junction.io.ByteLogReceiver;
import org.littletonrobotics.junction.io.ByteLogReplay;
import org.littletonrobotics.junction.io.LogSocketServer;
*/


//import team3176.robot.commands.drivetrain.SwerveDrive;
import team3176.robot.constants.LoggerConstants;
import team3176.robot.constants.RobotConstants;
//import team3176.robot.subsystems.*;
//import team3176.robot.subsystems.angler.Angler;
//import team3176.robot.subsystems.clarke.Clarke;
//import team3176.robot.subsystems.climb.Climb;
//import team3176.robot.subsystems.drivetrain.Drivetrain;
//import team3176.robot.subsystems.feeder.Feeder;
//import team3176.robot.subsystems.flywheel.Flywheel;
//import team3176.robot.subsystems.indexer.Indexer;
import team3176.robot.subsystems.intake.Intake;
import team3176.robot.subsystems.vision.Vision;
import team3176.robot.util.God.Alert;
import team3176.robot.util.God.Alert.AlertType;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  //private Intake m_Intake;
  //private Climb m_Climb; //
  //private Indexer m_Indexer;
  //private Angler m_Angler;
  //private Flywheel m_Flywheel;
  //private Feeder m_Feeder;
  //private Drivetrain m_Drivetrain;
  //private Controller m_Controller;
  private Vision m_Vision;
  //private Clarke m_Clarke;
  //private AnalogPotentiometer m_pressureSensor;

   private final Alert logNoFileAlert =
      new Alert("No log path set for current robot. Data will NOT be logged.",
          AlertType.WARNING);
  private final Alert logReceiverQueueAlert =
      new Alert("Logging queue exceeded capacity, data will NOT be logged.",
          AlertType.ERROR);
  private final Alert logOpenFileAlert = new Alert(
      "Failed to open log file. Data will NOT be logged.", AlertType.ERROR);
  private final Alert logWriteAlert =
      new Alert("Failed write to the log file. Data will NOT be logged.",
          AlertType.ERROR);
  private final Alert sameBatteryAlert =
      new Alert("The battery has not been changed since the last match.",
          AlertType.WARNING);


  public Robot() {
    super(RobotConstants.loopPeriodSecs);
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    /* 
    Logger logger = Logger.getInstance();
    setUseTiming(Constants.getMode() != Mode.REPLAY);
    LoggedNetworkTables.getInstance()
        .addTable("/SmartDashboard/TunableNumbers");
    logger.recordMetadata("Robot", Constants.getRobot().toString());
    logger.recordMetadata("BatteryName", BatteryTracker.scanBattery(1.0));
    logger.recordMetadata("TuningMode", Boolean.toString(Constants.tuningMode));
    logger.recordMetadata("RuntimeType", getRuntimeType().toString());
    logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        logger.recordMetadata("GitDirty", "Unknown");
        break;
    }
    */

    if(LoggerConstants.IS_LOGGER_ACTIVE) {
      setUseTiming(isReal()); // Run as fast as possible during replay
      Logger m_logger = Logger.getInstance();
      m_logger.recordMetadata("Robot", RobotConstants.getRobot().toString());
      m_logger.recordMetadata("TuningMode", Boolean.toString(RobotConstants.tuningMode));
      m_logger.recordMetadata("RuntimeType", getRuntimeType().toString());
      m_logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
      m_logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
      m_logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
      m_logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
      m_logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
      switch (BuildConstants.DIRTY) {
        case 0:
          m_logger.recordMetadata("GitDirty", "All changes committed");
          break;
        case 1:
          m_logger.recordMetadata("GitDirty", "Uncomitted changes");
          break;
        default:
          m_logger.recordMetadata("GitDirty", "Unknown");
          break;
      }

      switch (RobotConstants.getMode()) {
        case REAL:
          String folder = RobotConstants.logFolders.get(RobotConstants.getRobot());
          if (folder != null) {
            m_logger.addDataReceiver(new WPILOGWriter(folder));
          } else {
            logNoFileAlert.set(true);
          }
          m_logger.addDataReceiver(new NT4Publisher());
          LoggedPowerDistribution.getInstance();
          break;

         case SIM:
          m_logger.addDataReceiver(new NT4Publisher());
          break;

      case REPLAY:
        String path = LogFileUtil.findReplayLog();
        m_logger.setReplaySource(new WPILOGReader(path));
        m_logger.addDataReceiver(
            new WPILOGWriter(LogFileUtil.addPathSuffix(path, "_sim")));
        break;
    }
    m_logger.start();


      /* OLD 2022 AdvantageKit Code for logging.  TODO: RIPE FOR REMOVAL 
      LoggedNetworkTables.getInstance().addTable("/SmartDashboard"); // Log & replay "SmartDashboard" values (no tables are logged by default).
      Logger.getInstance().recordMetadata("ProjectName", "MyProject"); // Set a metadata value
      
      if (isReal()) {
        Logger.getInstance().addDataReceiver(new ByteLogReceiver("/media/sda1/")); // Log to USB stick (name will be selected automatically)
        Logger.getInstance().addDataReceiver(new LogSocketServer(5800)); // Provide log data over the network, viewable in Advantage Scope.
      } else {
        String path = ByteLogReplay.promptForPath(); // Prompt the user for a file path on the command line
        Logger.getInstance().setReplaySource(new ByteLogReplay(path)); // Read log file for replay
        Logger.getInstance().addDataReceiver(new ByteLogReceiver(ByteLogReceiver.addPathSuffix(path, "_sim"))); // Save replay results to a new log with the "_sim" suffix

        Logger.getInstance().start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
      } 
      */
      }

    //m_Intake = Intake.getInstance();
    //m_Indexer = Indexer.getInstance();
    //m_Angler = Angler.getInstance();
    //m_Flywheel = Flywheel.getInstance();
    //m_Feeder = Feeder.getInstance();
    //m_Drivetrain = Drivetrain.getInstance();
    //m_Controller = Controller.getInstance();
    m_Vision = Vision.getInstance();

    //m_pressureSensor = new AnalogPotentiometer(1/*, scale [ex: 250], offset[ex: -25]*/);

    m_Vision.setActivePipeline(2);
    //CameraServer.startAutomaticCapture(); //Fish-I Camera
    // CameraServer.startAutomaticCapture("Fish-I", 0);

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    /*
    SmartDashboard.putNumber("PSI", m_pressureSensor.get());
    SmartDashboard.putBoolean("Climb", m_pressureSensor.get() > 40);
    SmartDashboard.putBoolean("High Climb", m_pressureSensor.get() > 60);
    */

    if (LoggerConstants.IS_CMD_SCH_LOGGING_ACTIVE) {
      Logger.getInstance().recordOutput("Scheduler Commands", NetworkTableInstance.getDefault()
        .getEntry("/LiveWindow/Ungrouped/Scheduler/Names").getStringArray(new String[] {}));
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
   /* 
    m_Intake.mode = "disabled";
    m_Indexer.mode = "disabled";
    m_Angler.mode = "disabled";
    m_Flywheel.mode = "disabled";
    m_Feeder.mode = "disabled";
    */
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    /*
    m_Intake.mode = "auto";
    m_Indexer.mode = "auto";
    m_Angler.mode = "auto";
    m_Flywheel.mode = "auto";
    m_Feeder.mode = "auto";
    */
    
    //m_robotContainer.AutonInitRobotCentric();
    //*m_robotContainer.TelopInitFieldCentric();
    //*m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    /*
    m_Intake.mode = "teleop";
    m_Indexer.mode = "teleop";
    m_Angler.mode = "teleop";
    m_Flywheel.mode = "teleop";
    m_Feeder.mode = "teleop";
    */
    
    //*m_robotContainer.TelopInitFieldCentric();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
     if (Timer.getMatchTime() > 0 && Timer.getMatchTime() < 0.5) {
     }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    /*
    m_Intake.mode = "test";
    m_Indexer.mode = "test";
    m_Angler.mode = "test";
    m_Flywheel.mode = "test";
    m_Feeder.mode = "test";
    */

    /* 
    m_Drivetrain.setDefaultCommand(new SwerveDrive(
        () -> m_Controller.getForward(), 
        () -> m_Controller.getStrafe(),
        () -> m_Controller.getSpin()//,
    ));
    */
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    /*
    m_Intake.periodic();
    m_Indexer.periodic(); 
    m_Angler.periodic();
    m_Flywheel.periodic();
    m_Feeder.periodic();
    */
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
