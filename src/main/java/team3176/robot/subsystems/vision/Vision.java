// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.constants.VisionConstants;
import edu.wpi.first.math.controller.PIDController;
public class Vision extends SubsystemBase {
  
  private static Vision instance = new Vision();

  public NetworkTableInstance tableInstance;
  public NetworkTable limelightTable;
  public DoubleTopic tgtValid;
  public DoubleSubscriber tv;
  public DoubleTopic xOffset;
  public DoubleSubscriber tx;
  public DoubleTopic yOffset;
  public DoubleSubscriber ty;
  public DoubleTopic tShort;
  public DoubleSubscriber tshort;
  public DoubleTopic tLong;
  public DoubleSubscriber tlong;
  public DoubleTopic tHor;
  public DoubleSubscriber thor;
  public DoubleTopic tVert;
  public DoubleSubscriber tvert;
  public DoubleArrayTopic tCornXY;
  public DoubleArraySubscriber tcornxy;
  private DoubleTopic tL;
  private DoubleSubscriber tl;
  private DoubleTopic Pipeline;
  private DoubleSubscriber pipeline;
  private DoubleTopic camMODE;
  private DoubleSubscriber camMode;
  private DoubleTopic LEDMode;
  private DoubleSubscriber ledMode;
  private DoublePublisher camSetter;
  private DoublePublisher ledSetter;
  public DoublePublisher pipeSetter;
  public DoubleTopic idTopic;
  public DoubleSubscriber idSub;
    
  private double activePipeline = 1;
  private double startTime;

  private double deltaXCam;
  private double radius;

  // initializing variables for kinematic calculations
  private final double gravity = 9.81; // m/s^2
  private double deltaX; // m
  private double deltaY; // m
  private double initialVelocity; // m/s
  public double[] initialAngle = {45, 50, 55, 60, 65, 70, 75, 80, 85, 90}; // deg from horizontal
  private double finalAngle; // radians
  private double xVelocity; // m/s
  private double initialYVelocity; // m/s
  private double finalYVelocity; // m/s
  private double time; // seconds

  private ArrayList<Double> tcornx = new ArrayList<>(4);
  private ArrayList<Double> tcorny = new ArrayList<>(4);

  private double[] information = new double[2];

  private PIDController visionSpinLockPID;
  private boolean isVisionSpinCorrectionOn = false;
  private double visionSpinCorrection;

  //private int ballLocation = -999; // -999=no ball detected, 0=ball to left, 1=ball exactly 0 degrees forward, 2=ball to right
  //private double ballDegrees = -999; // degrees away from Limelight where ball is located. Positive = to left. Negative = to right. Zero = straight ahead.

  /**
   * Creates the default references for VisionClient, specifically for Limelight values
   */
  public Vision(){
    tableInstance = NetworkTableInstance.getDefault();
    limelightTable = tableInstance.getTable("limelight");
    updateVisionData();

    limelightTable.getEntry("pipeline").setNumber(activePipeline);

    tcornx.ensureCapacity(4);
    tcorny.ensureCapacity(4);

    visionSpinLockPID = new PIDController(0.01, 0.0, 0.0);
  }

  public static Vision getInstance(){
    if (instance == null) {
      instance = new Vision();
    }
    return instance;
  }

  /**
   * Can be called to force update of VisionClient data structure
   */
  public void updateVisionData(){
    tgtValid = limelightTable.getDoubleTopic("tv");
    tv = tgtValid.subscribe( 0.0);
    xOffset = limelightTable.getDoubleTopic("tx");
    tx = xOffset.subscribe(0.0);
    yOffset = limelightTable.getDoubleTopic("ty");
    ty = yOffset.subscribe(0.0);
    tShort = limelightTable.getDoubleTopic("tshort");
    tshort = tShort.subscribe(0.0);
    tLong = limelightTable.getDoubleTopic("tlong");
    tlong = tLong.subscribe(0.0);
    tHor = limelightTable.getDoubleTopic("thor");
    thor = tHor.subscribe(0.0);
    tVert = limelightTable.getDoubleTopic("tvert");
    tvert = tVert.subscribe(0.0);
    tCornXY = limelightTable.getDoubleArrayTopic("tcornxy");
    tcornxy = tCornXY.subscribe(new double[]{});
    tL = limelightTable.getDoubleTopic("tl");
    tl = tL.subscribe(0.0);
    Pipeline = limelightTable.getDoubleTopic("pipeline");
    pipeline = Pipeline.subscribe(0.0);
    camMODE = limelightTable.getDoubleTopic("camMode");
    camMode = camMODE.subscribe(0.0);
    LEDMode = limelightTable.getDoubleTopic("ledMode");
    ledMode = LEDMode.subscribe(0.0);
    activePipeline = pipeline.get();
    camSetter = camMODE.publish();
    pipeSetter = Pipeline.publish();
    ledSetter = LEDMode.publish();
    idTopic = limelightTable.getDoubleTopic("tid");
    idSub = idTopic.subscribe(0.0);
  }
  public double getTargetID(){
    return idSub.get();
  }
  public double getLEDState(){
    double LEDState = ledMode.get();
    return LEDState;
  }
  public void targetRecogControlLoop(){
    // used to calculate latency
    startTime = Timer.getFPGATimestamp();

    updateVisionData();

    separateCornArray();
    SmartDashboard.putBoolean("Has Run?", true);
    SmartDashboard.putBoolean("Empty?", tcornx.isEmpty());
    SmartDashboard.putNumber("Info", tcornx.isEmpty() ? 0 : tcornx.size());
    if(tcornx.size() < 4 || tcornx.size() > 5){
      return;
    }

    deltaXCam = findDeltaX();
    SmartDashboard.putNumber("Delta X Cam", deltaXCam);

    calculateTargetDistance();

    // get the initial velocity and angle of ball
    information = findInitialAngleAndVelocity((int) Math.ceil(initialAngle.length / 2));

    publishAllData();

    SmartDashboard.putNumber("Latency (ms)", ((Timer.getFPGATimestamp() - startTime) * 1000) + tl.get() + 11);
  }

  public void separateCornArray(){
    if(!tcornx.isEmpty() && !tcorny.isEmpty()){
      tcornx.clear();
      tcorny.clear();
    }
    for(int i = 0; i < 4; i++){
      tcornx.add(limelightTable.getEntry("tx" + i).getDouble(-999.0));
      tcorny.add(limelightTable.getEntry("ty" + i).getDouble(-999.0));
    }
    tcornx.remove(-999.0);
    tcorny.remove(-999.0);
  }

  public double findDeltaX(){
    Double[] tempCorn = tcornx.toArray(new Double[0]);
    Arrays.sort(tempCorn);
    return tempCorn[tempCorn.length - 1] - tempCorn[0];
  }

  public void calculateTargetDistance(){
    double rawDistance = (VisionConstants.VISION_CONSTANT / deltaXCam);
    SmartDashboard.putNumber("Raw Distance", rawDistance);
    deltaX = rawDistance * Math.cos(VisionConstants.cameraAngle * VisionConstants.DEG2RAD); //+ (10 * VisionConstants.INCHES2METERS);
    deltaY = rawDistance * Math.sin(VisionConstants.cameraAngle * VisionConstants.DEG2RAD);
    SmartDashboard.putNumber("DeltaX", deltaX);
    SmartDashboard.putNumber("DeltaY", deltaY);
  }

  public double[] findInitialAngleAndVelocity(int angleIdx){
    if(angleIdx >= initialAngle.length || angleIdx < 0){
      return null;
    }
    
    findInitialVelocity(angleIdx);
    
    // Haha, get it? Because this variable "doublechecks" the result. Programming jokes are the best. 
    double check = -Math.sqrt(Math.pow(initialVelocity * Math.sin(initialAngle[angleIdx]), 2) + 2 * gravity * deltaY);
    
    if(check > 0){
      time = (check - initialVelocity) / gravity;
      double fullCalculatedDistance = initialVelocity * Math.cos(initialAngle[angleIdx]) * time;
      if(fullCalculatedDistance > deltaX){
        return findInitialAngleAndVelocity(angleIdx + 1);
      } else{
        return findInitialAngleAndVelocity(angleIdx - 1);
      }
    }
    
    solveOtherVariablesFromVelocity(angleIdx);
    
    double[] arrayToSend = {initialVelocity, initialAngle[angleIdx]};
    return arrayToSend;
  }
  
  private void findInitialVelocity(int angleIdx){
    double term1 = deltaX / (Math.cos(initialAngle[angleIdx]));
    double term2 = -gravity / (2 * (deltaX * Math.tan(initialAngle[angleIdx]) - deltaY));
    initialVelocity = term1 * Math.sqrt(term2);
  }
  
  private void solveOtherVariablesFromVelocity(int angleIdx){
    xVelocity = initialVelocity * Math.cos(initialAngle[angleIdx]);
    initialYVelocity = initialVelocity * Math.sin(initialAngle[angleIdx]);
    finalYVelocity = Math.sqrt(Math.pow(initialYVelocity, 2) + 2 * -gravity * deltaY);
    time = deltaX / xVelocity;
  }

  public void setVisionProcessing(boolean imageProcessing){
    if(imageProcessing){
      camSetter.set(0);
    } else{
      camSetter.set(1);
    }
  }

  public boolean getVisionProcessing(){
    return camMode.get() == 0.0;
  }

  public void setActivePipeline(int newPipeline){
    if(newPipeline > -1 && newPipeline < 4){
      activePipeline = newPipeline;
      pipeSetter.set(activePipeline);
    } else{
      System.out.println("Invalid Pipeline Requested, No Change Was Made");
    }
  }

  private void publishAllData(){
    SmartDashboard.putNumber("initialVelocity", initialVelocity);

    SmartDashboard.putBoolean("Has Targets", (tv.get() == 1));
    SmartDashboard.putNumber("tshort", tshort.get());
    SmartDashboard.putNumber("tvert", tvert.get());

    double numTargets = tcornx.size();
    SmartDashboard.putNumber("Number of Targets", numTargets);

    SmartDashboard.putNumber("Radius", radius);
    SmartDashboard.putNumber("Horizontal Distance", deltaX);
    SmartDashboard.putNumber("Vertical Distance", deltaY);

    SmartDashboard.putNumber("Distance According to Camera", deltaXCam);

    SmartDashboard.putNumber("Approx. Latency (ms)", ((Timer.getFPGATimestamp() - startTime) * 1000) + tl.get() + 11);
  }

  public double getCurrentPipeline(){
    return pipeline.get();
  }

  public void switchLEDs(LEDState newLEDState){
    if(newLEDState == LEDState.OFF){
      ledSetter.set(1);
    } else if(newLEDState == LEDState.ON){
      ledSetter.set(3);
    } else if(newLEDState == LEDState.BLINK){
      ledSetter.set(2);
    } else{
      System.out.println("Invalid LED State Requested, No Change Made");
    }
  }

  public enum LEDState {
    OFF, ON, BLINK, NULL
  }

  ArrayList<Double> testValues = new ArrayList<Double>();

  /*public void averageMeasurements(double newValue){
    testValues.add(newValue);
    double total = 0;
    for(double value : testValues) {
      total += value;
    }
    SmartDashboard.putNumber("Average Distance", total / testValues.size());
    System.out.println("DONE!");
  }*/

  public double[] getVisionInformation(){
    return information;
  }

  public double[] TestVisionKinematics(int angleIdx, double testDeltaX, double testDeltaY){
    if(angleIdx >= initialAngle.length || angleIdx < 0){
      SmartDashboard.putBoolean("Has Run?", true);
      return null;
    }
    
    double term1 = testDeltaX / (Math.cos(initialAngle[angleIdx]));
    double term2 = gravity / (2 * ((testDeltaX * Math.tan(initialAngle[angleIdx])) - testDeltaY));
    double testInitialVelocity = term1 * Math.sqrt(term2);
    SmartDashboard.putNumber("Term 1", term1);
    SmartDashboard.putNumber("Term 2", term2);
    SmartDashboard.putNumber("TestInitialVelocity", testInitialVelocity);
    
    // Haha, get it? Because this variable "doublechecks" the result. Programming jokes are the best. 
    double check = -Math.sqrt(Math.pow(testInitialVelocity * Math.sin(initialAngle[angleIdx]), 2) + 2 * -gravity * testDeltaY);
    SmartDashboard.putNumber("Check", check);
    
    if(check > 0){
      double timeCheck = (check - testInitialVelocity) / gravity;
      double fullCalculatedDistance = testInitialVelocity * Math.cos(initialAngle[angleIdx]) * timeCheck;
      if(fullCalculatedDistance > testDeltaX){
        return TestVisionKinematics(angleIdx + 1, testDeltaX, testDeltaY);
      } else{
        return TestVisionKinematics(angleIdx - 1, testDeltaX, testDeltaY);
      }
    }

    double testXVelocity = testInitialVelocity * Math.cos(initialAngle[angleIdx]);
    double testInitialYVelocity = testInitialVelocity * Math.sin(initialAngle[angleIdx]);
    double testFinalYVelocity = Math.sqrt(Math.pow(testInitialYVelocity, 2) + 2 * -gravity * testDeltaY);
    double testTime = testDeltaX / testXVelocity;

    SmartDashboard.putNumber("Test X Velocity", testXVelocity);
    SmartDashboard.putNumber("Test Initial Y Velocity", testInitialYVelocity);
    SmartDashboard.putNumber("Test Final Y Velocity", testFinalYVelocity);
    SmartDashboard.putNumber("Test Time", testTime);

    double[] arrayToSend = {testInitialVelocity, initialAngle[angleIdx]};
    return arrayToSend;
  }

  public boolean getIsVisionSpinCorrectionOn(){
    return this.isVisionSpinCorrectionOn;
  }

  public void setVisionSpinCorrection(boolean onOrOff){
    this.isVisionSpinCorrectionOn = onOrOff;
    SmartDashboard.putBoolean("VisionSpinCorrectionOn", isVisionSpinCorrectionOn);
  }
  public void setVisionSpinCorrectionOn(){
    if (this.tv.get() == 1) { 
      setVisionSpinCorrection(true);
      SmartDashboard.putBoolean("VisionSpinCorrectionOn", true);
    }
  }
  
  public void setVisionSpinCorrectionOff(){
    setVisionSpinCorrection(false);
    SmartDashboard.putBoolean("VisionSpinCorrectionOn", false);
  }


  public void toggleVisionSpinCorrectionOnOff(){
    setVisionSpinCorrection(!isVisionSpinCorrectionOn);
    SmartDashboard.putBoolean("VisionSpinCorrectionOn", isVisionSpinCorrectionOn);
  }

  public double getVisionSpinCorrection() {
    updateVisionData();
    this.visionSpinCorrection = 1 * visionSpinLockPID.calculate(this.tx.get(), 0);
    return visionSpinCorrection;
  }
}