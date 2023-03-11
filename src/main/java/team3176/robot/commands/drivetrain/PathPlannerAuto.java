package team3176.robot.commands.drivetrain;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
//import team3176.robot.commands.superstructure.intake.IntakeExtendFreeSpin;
import team3176.robot.constants.DrivetrainConstants;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.drivetrain.Drivetrain.driveMode;
//import team3176.robot.subsystems.superstructure.*;
//import team3176.robot.subsystems.superstructure.Claw;
//import team3176.robot.subsystems.superstructure.Superstructure;

public class PathPlannerAuto {
    Command auto;
    public PathPlannerAuto(String autoPathName, Command doBefore) {
        //Claw m_Claw = Claw.getInstance();
        Drivetrain driveSubsystem = Drivetrain.getInstance();
        //Superstructure m_Superstructure = Superstructure.getInstance();
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(autoPathName, new PathConstraints(2, 1.3));
        //System.out.println("length" + pathGroup.size());
        // This is just an example event map. It would be better to have a constant, global event map
        // in your code that will be used by all path following commands.
        HashMap<String, Command> eventMap = new HashMap<>();
        //eventMap.put("scoreHighFirst", m_Superstructure.scoreFirstGamePieceAuto());
        eventMap.put("autoBalance", new AutoBalance().andThen(new SwerveDefense()).finallyDo((b) -> {
            driveSubsystem.setDriveMode(driveMode.DEFENSE);
            driveSubsystem.drive(0.0,0.0,0.0);
        }));
        //eventMap.put("groundCube",m_Superstructure.groundCube());
        // eventMap.put("intakeDown", new IntakeDown());
        // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            driveSubsystem::getPose,
            driveSubsystem::resetPose,
            DrivetrainConstants.DRIVE_KINEMATICS,
            new PIDConstants(5.0,0.0,0.0),
            new PIDConstants(1.5,0.0,0.0),
            driveSubsystem::setModuleStates,
            eventMap,
            true,
            driveSubsystem);
        if (doBefore != null){
            auto = doBefore.andThen(autoBuilder.fullAuto(pathGroup));
        } else {
            auto = autoBuilder.fullAuto(pathGroup);
        }
        
    }
    public PathPlannerAuto(String autoPathName) {
        this(autoPathName,null);
    }
    public Command getauto(){
        return auto;
    }

}
