// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import team3176.robot.subsystems.RobotStateIO; 
import team3176.robot.subsystems.RobotStateIO.RobotStateIOInputs; 

import team3176.robot.subsystems.signalling.Signalling;
import team3176.robot.subsystems.intake.Intake;
import team3176.robot.subsystems.claw.Claw;
import team3176.robot.constants.RobotConstants.Status;



public class RobotState extends SubsystemBase {

  private final RobotStateIO io;
  private final RobotStateIOInputs inputs = new RobotStateIOInputs();
  private static RobotState instance;
  private int wantedGPState;
  private Signalling m_Signalling;
  private Intake m_Intake;
  private Claw m_Claw;

  private Alliance alliance; 

  public enum e_CentricWantedState {
    ROBOT,
    FIELD
  }

  public enum e_ClawPositionState {
    OPEN,
    CLOSED,
    IDLE
  }

  public enum e_ClawRollersState {
    PosSPIN,  //Means front-most side of rollers spinning INTO the Claw, toward it's center 
    NegSPIN,  //Means front-most side of rollers spinning OUT-OF the Claw, toward it's exterior
    NoSPIN,  //Means front-most rollers are not spinning
  }

  public enum e_IntakePositionState {
    EXTENDED,
    RETRACTED
  }

  public enum e_IntakeDetectsImHolding {
    CONE,
    CUBE,
    NOTHING,
    ERROR
  }

  enum e_IntakeMotorState {
    PosSPIN,  //Means front-most side of rollers spinning INTO the Intake, toward it's center 
    NegSPIN,  //Means front-most side of rollers spinning OUT-OF the Intake, toward it's exterior
    NoSPIN,  //Means front-most rollers are not spinning
  }

  enum e_ArmShoulderPositionState {
    IsUP,  //Means Elevator is in up postion
    IsDOWN  // Means Elevator is in down position
  }

  public enum e_ArmElbowPositionState {
    IsPICKUP,   //Means Arms is in position to receive game element from Intake
    IsHIGHCONE,  // Means Arm is in High Position to deposit cone
    IsHIGHCUBE,  // Means Arm is in High Position to deposit cube
    IsMIDCONE,  // Means Arm is in Mid Position to deposit cone
    IsMIDCUBE,  // Means Arm is in Mid Position to deposit cube
    IsFLOORCONE,  // Means Arm is in Floor High Position to deposit cone 
    IsFLOORCUBE,  // Means Arm is in Floor Position to deposit cube
  }

  public enum e_CurrentGameElementImWanting{
    CONE,
    CUBE,
    NONE
  }

  public enum e_CurrentGameElementImHolding{
    CONE,
    CUBE,
    NONE
  }


  private RobotState(RobotStateIO io) {
    this.io = io;
    m_Signalling = Signalling.getInstance();
    m_Intake = Intake.getInstance();
    m_Claw = Claw.getInstance();
    wantedGPState = 0;
  }

  public void update() {
    if (DriverStation.isFMSAttached() && (alliance == null)) {
      alliance = DriverStation.getAlliance();
    }
  }

  public void setColorWantState()
  {
    System.out.println("WAS CALLED");
    wantedGPState++; // wantedGPState tells RobotState what GamePiece the robot wants: 3 = NONE, 2 = CONEFLASH, 1 = CUBEFLASH
    if (wantedGPState == 3)
    {
      m_Signalling.setleft(e_CurrentGameElementImWanting.NONE);
    }
    else if (wantedGPState == 2)
    {
      m_Signalling.setleft(e_CurrentGameElementImWanting.CONE);
    }
    else if (wantedGPState == 1)
    {
      m_Signalling.setleft(e_CurrentGameElementImWanting.CUBE);
    }
    if (wantedGPState == 3) {wantedGPState = 0;}
  }

  public void setGPHoldingState()
  {
    if (m_Intake.getProximity() <= 150)
    {
      if (wantedGPState == 1)
      {
        m_Signalling.setleft(e_CurrentGameElementImHolding.CONE);
      }
      else if (wantedGPState == 2)
      {
        m_Signalling.setleft(e_CurrentGameElementImHolding.CUBE);
      }
    }
    else if (m_Intake.getLinebreak() == true)
    {
      m_Signalling.setleft(e_CurrentGameElementImHolding.NONE);
      wantedGPState = 0;
    }
  }

  public void setIntakeHoldingState()
  {
    if (m_Intake.isConeThere() == true)
    {
      m_Signalling.setIntakeGPColor(e_IntakeDetectsImHolding.CONE);
    }
    if (m_Intake.isCubeThere() == true)
    {
      m_Signalling.setIntakeGPColor(e_IntakeDetectsImHolding.CUBE);
    }
    if (m_Intake.isConeThere() == false && m_Intake.isCubeThere() == false)
    {
      m_Signalling.setIntakeGPColor(e_IntakeDetectsImHolding.NOTHING);
    }
    if (m_Intake.isConeThere() == true && m_Intake.isCubeThere() == true)
    {
      m_Signalling.setIntakeGPColor(e_IntakeDetectsImHolding.ERROR);
    }
  }

  public void CentricState()
  {
    // if FieldCentric, then light up one color
  }

  public Command setColorWantStateCommand()
  {
    return this.runOnce(() -> setColorWantState());
  }

  public static RobotState getInstance() {
    if(instance == null) {instance = new RobotState(new RobotStateIO() {});}
    return instance;
  }


  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Intake", inputs);
    
    for (int i = 0; i < 51; i++)
    {
      if (i == 50)
      {
        setGPHoldingState();
        //setIntakeHoldingState();
      }
    }
  }

  @Override
  public void simulationPeriodic() {}
}