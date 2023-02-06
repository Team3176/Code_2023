// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.drivetrain;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.*;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import team3176.robot.util.God.PID3176;
import team3176.robot.constants.DrivetrainConstants;
import team3176.robot.constants.SwervePodHardwareID;
import team3176.robot.constants.SwervePodConstants2022;
// import team3176.robot.util.God.PID3176;
import team3176.robot.subsystems.drivetrain.SwervePod;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.subsystems.controller.Controller;


import org.littletonrobotics.junction.Logger;
import team3176.robot.subsystems.drivetrain.DrivetrainIO.DrivetrainIOInputs;



  

public class Drivetrain extends SubsystemBase {
  private static Drivetrain instance;
  private AHRS m_NavX;
  public SwerveDriveOdometry odom;
  public SwerveDrivePoseEstimator poseEstimator;

  public NetworkTableInstance inst;
  public NetworkTable table;
  public DoubleTopic dblTopic;
  public DoublePublisher dblPub;
  //private Controller controller = Controller.getInstance();
  //private Vision m_Vision = Vision.getInstance();
  public enum coordType {
    FIELD_CENTRIC, ROBOT_CENTRIC
  }
  public coordType currentCoordType = coordType.FIELD_CENTRIC; 
 // private PowerDistribution PDP = new PowerDistribution(PowerManagementConstants.PDP_CAN_ID, ModuleType.kCTRE);

  private ArrayList<SwervePod> pods;

  private driveMode currentDriveMode = driveMode.DRIVE;

  public TalonFX[] driveControllers = { new TalonFX(DrivetrainConstants.FR.THRUST_CID),
      new TalonFX(DrivetrainConstants.FL.THRUST_CID), new TalonFX(DrivetrainConstants.BL.THRUST_CID),
      new TalonFX(DrivetrainConstants.BR.THRUST_CID) };

  public CANSparkMax[] azimuthControllers = { new CANSparkMax(DrivetrainConstants.STEER_FR_CID, MotorType.kBrushless),
      new CANSparkMax(DrivetrainConstants.STEER_FL_CID, MotorType.kBrushless), new CANSparkMax(DrivetrainConstants.STEER_BL_CID, MotorType.kBrushless),
      new CANSparkMax(DrivetrainConstants.STEER_BR_CID, MotorType.kBrushless) };

  Rotation2d FieldAngleOffset = Rotation2d.fromDegrees(0.0);

  private double relMaxSpeed;

  private double forwardCommand;
  private double strafeCommand;
  private double spinCommand;
  private double spinCommandInit;

  //spin lock
  private PIDController spinLockPID;
  private Rotation2d spinLockAngle = Rotation2d.fromDegrees(0.0);
  private boolean isSpinLocked = false;

  private boolean isTurboOn = false;

  private int arraytrack;
  double[] angleHist = { 0.0, 0.0, 0.0, 0.0, 0.0 };
  double angleAvgRollingWindow;

  public enum driveMode {
    DEFENSE, DRIVE, VISION
  }

  private SwervePod podFR;
  private SwervePod podFL;
  private SwervePod podBL;
  private SwervePod podBR;

  NetworkTable vision;
  DoubleArraySubscriber vision_pose;
  private final DrivetrainIO io;
  //private final DrivetrainIOInputs inputs = new DrivetrainIOInputs();

  private Drivetrain(DrivetrainIO io) {
    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("datatable");
    
    dblTopic = table.getDoubleTopic("Angle");

    dblPub = dblTopic.publish();
    
    this.io = io;
    
    //check for duplicates
    assert(!SwervePodHardwareID.check_duplicates_all(DrivetrainConstants.FR,DrivetrainConstants.FL,DrivetrainConstants.BR,DrivetrainConstants.BL));
    // Instantiate pods
    
    podFR = new SwervePod(0, driveControllers[0], azimuthControllers[0]);
    podFL = new SwervePod(1, driveControllers[1], azimuthControllers[1]);
    podBL = new SwervePod(2, driveControllers[2], azimuthControllers[2]);
    podBR = new SwervePod(3, driveControllers[3], azimuthControllers[3]);

    // Instantiate array list then add instantiated pods to list
    pods = new ArrayList<SwervePod>();
    pods.add(podFR);
    pods.add(podFL);
    pods.add(podBL);
    pods.add(podBR);



    
    m_NavX = new AHRS(SPI.Port.kMXP);

    odom = new SwerveDriveOdometry(DrivetrainConstants.DRIVE_KINEMATICS, this.getChassisYaw(), new SwerveModulePosition[] {
      podFR.getPosition(),
      podFL.getPosition(),
      podBL.getPosition(),
      podBR.getPosition()
    }, new Pose2d(0.0, 0.0, new Rotation2d()));
    poseEstimator = new SwerveDrivePoseEstimator(DrivetrainConstants.DRIVE_KINEMATICS, getChassisYaw(), getSwerveModulePositions(), odom.getPoseMeters());
    //TODO: update covariance matrix for vision
    //poseEstimator.setVisionMeasurementStdDevs(new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1,0.1,0.01));
    spinLockPID = new PIDController(0.15, 0.0, 0.0);
    //set for max and min of degrees for Rotation2D
    spinLockPID.enableContinuousInput(-180,180);

    arraytrack = 0;
    angleAvgRollingWindow = 0;

    // TODO: We initialize to face forward but how do we make this into a command?
    // Maybe we say drive with the below parameters, but where?
    /*
     * // Start wheels in a forward facing direction
     */

    this.forwardCommand = Math.pow(10, -15); // Has to be positive to turn that direction?
    this.strafeCommand = 0.0;
    this.spinCommand = 0.0;
    vision = NetworkTableInstance.getDefault().getTable("limelight");
    vision_pose = table.getDoubleArrayTopic("botpose").subscribe(new double[6]);

  }

  // Prevents more than one instance of drivetrian
  public static Drivetrain getInstance() {
    if(instance == null) {instance = new Drivetrain(new DrivetrainIO() {});}
    return instance;
  }

  /**
   * public void drive(double forwardCommand, double strafeCommand, double
   * spinCommand, int uselessVariable) { double smallNum = Math.pow(10, -15);
   * //spinCommand = (spinCommand - (-1))/(1 - (-1)); //rescales spinCommand to a
   * 0..1 range double angle = (spinCommand * Math.PI) + Math.PI; // <- diff coord
   * system than -1..1 = 0..2Pi // This coord system is 0..1 = Pi..2Pi, & // 0..-1
   * = Pi..-2PI // right? // Fixed by new rescaling at line 140?
   * pods.get(0).set(smallNum, angle); }
   */
  
   /**
    * public facing drive command that allows command to specify if the command is field centric or not
    * @param forwardCommand meters per second
    * @param strafeCommand meters per second
    * @param spinCommand meters per second
    * @param type  FIELD CENTRIC or ROBOT_CENTRIC
    */
  public void drive(double forwardCommand, double strafeCommand, double spinCommand, coordType type) {
    ChassisSpeeds speed = new ChassisSpeeds(forwardCommand,strafeCommand,spinCommand);
    if(type == coordType.FIELD_CENTRIC) {
      speed = ChassisSpeeds.fromFieldRelativeSpeeds(speed, this.getChassisYaw());
    }
    p_drive(speed.vxMetersPerSecond, speed.vyMetersPerSecond, speed.omegaRadiansPerSecond);
  }
  /**
   * default call will assume robot_centric
   * @param forwardCommand
   * @param strafeCommand
   * @param spinCommand
   */
  public void drive(double forwardCommand, double strafeCommand, double spinCommand) {
    drive(forwardCommand, strafeCommand, spinCommand, currentCoordType);
  }
   /**
   * 
   * @param forwardCommand meters per second
   * @param strafeCommand  meters per second
   * @param spinCommand    meters per second
   */
  private void p_drive(double forwardCommand, double strafeCommand, double spinCommand) {
    this.spinCommandInit = spinCommand;
    this.forwardCommand = forwardCommand;
    this.strafeCommand = strafeCommand;  // TODO: The y is inverted because it is backwards for some reason, why?
    this.spinCommand = spinCommand;
    //System.out.println("forward: "+ forwardCommand + "strafe: " + strafeCommand + "spin: " + spinCommand);
    // if (!isTurboOn) {
    //   this.forwardCommand *= DrivetrainConstants.NON_TURBO_PERCENT_OUT_CAP;
    //   this.strafeCommand *= DrivetrainConstants.NON_TURBO_PERCENT_OUT_CAP;
    //   //this.spinCommand *= DrivetrainConstants.NON_TURBO_PERCENT_OUT_CAP;
    //   this.spinCommand *= DrivetrainConstants.NON_TURBO_PERCENT_OUT_CAP;
    // } else {
    //   this.spinCommand *= 2; 
    // }
    if(isSpinLocked) {
      this.spinCommand = spinLockPID.calculate(getChassisYaw().getDegrees(), spinLockAngle.getDegrees());
    }

    calculateNSetPodPositions(this.forwardCommand, this.strafeCommand, this.spinCommand);
    
  }

  /**
   * Robot Centric Forward, strafe, and spin to set individual pods commanded spin speed and drive speed
   * @param forwardCommand meters per second
   * @param strafeCommand  meters per second
   * @param spinCommand    meters per second
   */
  private void calculateNSetPodPositions(double forwardCommand, double strafeCommand, double spinCommand) {

    if (currentDriveMode != driveMode.DEFENSE) {
      // Create arrays for the speed and angle of each pod
      // double[] podDrive = new double[4];
      // double[] podSpin = new double[4];
      ChassisSpeeds curr_chassisSpeeds = new ChassisSpeeds(forwardCommand,strafeCommand,spinCommand);
      SwerveModuleState[] pod_states = DrivetrainConstants.DRIVE_KINEMATICS.toSwerveModuleStates(curr_chassisSpeeds);
      for (int idx = 0; idx < (pods.size()); idx++) {
          
            pods.get(idx).set_module(pod_states[idx]);
          
      }
      SmartDashboard.putNumber("spinCommand", spinCommand);
      SmartDashboard.putNumber("pod0 m/s", pod_states[0].speedMetersPerSecond);
      
    } else if (currentDriveMode == driveMode.DEFENSE) { // Enter defensive position
      double smallNum = Math.pow(10, -5);
      pods.get(0).set(smallNum, Rotation2d.fromRadians(1.0 * Math.PI / 8.0));
      pods.get(1).set(smallNum,  Rotation2d.fromRadians(-1.0 * Math.PI / 8.0));
      pods.get(2).set(smallNum,  Rotation2d.fromRadians(-3.0 * Math.PI / 8.0));
      pods.get(3).set(smallNum,  Rotation2d.fromRadians(3.0 * Math.PI / 8.0));
    }
  }

  public void stopMotors() {
    //TODO: this seems to voilate a data flow overiding pods and could cause issues should be a state variable
    for (int idx = 0; idx < (pods.size()); idx++) {
      driveControllers[idx].set(ControlMode.PercentOutput, 0);
      azimuthControllers[idx].set(0);
    }

  }

  public void setPodsAzimuthHome() {
      double smallNum = Math.pow(10, -5);
      pods.get(0).set(smallNum, Rotation2d.fromRadians(0.0));
      pods.get(1).set(smallNum,  Rotation2d.fromRadians(0.0));
      pods.get(2).set(smallNum,  Rotation2d.fromRadians(0.0));
      pods.get(3).set(smallNum,  Rotation2d.fromRadians(0.0));
  }

  public void sendPodsAzimuthToHome() {
    for (int idx = 0; idx < (pods.size()); idx++) {
      pods.get(idx).goHome();
    }
  }

  public void setCurrentPodPosAsHome() {
    for (int idx = 0; idx < (pods.size()); idx++) {
      pods.get(idx).setCurrentPositionAsHome();
    }
  }


  public void setDriveMode(driveMode wantedDriveMode) {
    currentDriveMode = wantedDriveMode;
  }

  
  public driveMode getCurrentDriveMode() {
    return currentDriveMode;
  }
  public Pose2d getPose() {
    return odom.getPoseMeters();
  }
  public void resetPose(Pose2d pose) {
    odom.resetPosition(new Rotation2d(), new SwerveModulePosition[] {
      podFR.getPosition(),
      podFL.getPosition(),
      podBL.getPosition(),
      podBR.getPosition()}, pose);
  }
  public void setModuleStates(SwerveModuleState[] states) {
    for (int idx = 0; idx < (pods.size()); idx++) {
      pods.get(idx).set_module(states[idx]);
    }
  }
  /**
   * Sets Turbo mode on or off
   * @param onOrOff Passing a value of true sets Turbo on (ie isTurboOn = true), and passing value of false sets Turbo off (ie isTurboOn = false)
   */
  public void setTurbo(boolean onOrOff) {
    this.isTurboOn = onOrOff;
  }
  public void setCoordType(coordType c) {
    this.currentCoordType = c;
  }

  public void setSpinLock(boolean b) {
    this.isSpinLocked = b;
  }
  public void setSpinLockAngle() {
    this.spinLockAngle = getChassisYaw();
  }

  /**
   * 
   * @return returns the chassis yaw wrapped between -pi and pi
   */
  public Rotation2d getChassisYawWrapped() {
    //its ugly but rotation2d is continuos but I imagine most of our applications we want it bounded between -pi and pi
    return Rotation2d.fromRadians(MathUtil.angleModulus(m_NavX.getRotation2d().minus(this.FieldAngleOffset).getRadians())); 
  }
  /**
   * The unbounded angle 
   * @return Rotation2d of the yaw
   */
  public Rotation2d getChassisYaw() {
    return m_NavX.getRotation2d().minus(this.FieldAngleOffset); 
  }

  public void resetFieldOrientation(){
    //do not need to invert because the navx rotation2D call returns a NWU coordsys!
    this.FieldAngleOffset = m_NavX.getRotation2d();
  }

  public double getPodVelocity(int podID) {
    return pods.get(podID).getVelocity();
  }

  public double  getPodAzimuth(int podID) {
    return pods.get(podID).getAzimuth();
  }
  public SwerveModulePosition[] getSwerveModulePositions() {
    return new SwerveModulePosition[] {
      podFR.getPosition(),
      podFL.getPosition(),
      podBL.getPosition(),
      podBR.getPosition()
    };
  }


/*
  public ChassisSpeeds getChassisSpeed() {
    return DrivetrainConstants.DRIVE_KINEMATICS.toChassisSpeeds(podFR.getState(), podFL.getState(), podBL.getState(), podBR.getState());
  }
*/




  @Override
  public void periodic() {
    dblPub.set(3.0);
    for (TimestampedDoubleArray v : vision_pose.readQueue()) {
      double[] vision_pose_array = v.value;
      Pose2d cam_pose =new Pose2d(vision_pose_array[0],vision_pose_array[1],Rotation2d.fromDegrees(vision_pose_array[5]));
      double xoffset = Units.inchesToMeters(285.16+ 40.45);
      double yoffset = Units.inchesToMeters(115.59 + 42.49);
      cam_pose = cam_pose.transformBy(new Transform2d(new Translation2d(xoffset,yoffset),new Rotation2d()));

      //update the pose estimator with correct timestamped values
      this.poseEstimator.addVisionMeasurement(cam_pose, v.timestamp);
      SmartDashboard.putNumber("camX",cam_pose.getX());
    }
    //update encoders
    this.poseEstimator.update(getChassisYaw(), getSwerveModulePositions());
    this.odom.update(getChassisYaw(), getSwerveModulePositions());
    
    // This method will be called once per scheduler every 500ms
    
    this.arraytrack++;
    if (this.arraytrack > 3) {
      this.arraytrack = 0;
    }
    
    SmartDashboard.putNumber("odomx", getPose().getX());
    SmartDashboard.putNumber("odomy", getPose().getY());
    SmartDashboard.putBoolean("Turbo", isTurboOn);
    // SmartDashboard.putBoolean("Defense", currentDriveMode == driveMode.DEFENSE);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
