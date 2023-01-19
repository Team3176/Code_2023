// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.util;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class instantPrintTest extends InstantCommand {
  private String stringToPrint;
  public instantPrintTest(String passThrough) {
    stringToPrint = passThrough;
  }

  @Override
  public void initialize() {
    System.out.println(stringToPrint);
  }
}
