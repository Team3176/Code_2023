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
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team3176.robot.constants.DrivetrainConstants;
import team3176.robot.subsystems.drivetrain.Drivetrain;

public class PathPlannerAuto {
    Command auto;
    public PathPlannerAuto(String autoPathName) {

        Drivetrain driveSubsystem = Drivetrain.getInstance();
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(autoPathName, new PathConstraints(2, 0.7));
        System.out.println("length" + pathGroup.size());
        // This is just an example event map. It would be better to have a constant, global event map
        // in your code that will be used by all path following commands.
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("print", new PrintCommand("action!"));
        eventMap.put("cube", new WaitCommand(1.0));
        eventMap.put("autoBalance", new AutoBalance().andThen(new SwerveDefense()));
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
    
        auto = autoBuilder.fullAuto(pathGroup);
    }
    public Command getauto(){
        return auto;
    }

}