// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.signalling;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Signalling extends SubsystemBase {
  
  private static Signalling instance = new Signalling();

  
  /**
   * Creates the default references for VisionClient, specifically for Limelight values
   */
  public Signalling(){

  }

  public static Signalling getInstance(){
    if (instance == null) {
      instance = new Signalling();
    }
    return instance;
  }

}
