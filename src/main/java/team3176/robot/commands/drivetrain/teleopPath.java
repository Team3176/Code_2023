package team3176.robot.commands.drivetrain;

import com.pathplanner.lib.*;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team3176.robot.constants.DrivetrainConstants;
import team3176.robot.subsystems.drivetrain.Drivetrain;

public class teleopPath extends CommandBase{
    Drivetrain m_Drivetrain;
    PathPlannerTrajectory traj1;
    PPSwerveControllerCommand swerveCommand;
    public teleopPath() {
        // super( null, 
        // Drivetrain.getInstance()::getPose, // Pose supplier
        // DrivetrainConstants.DRIVE_KINEMATICS, // SwerveDriveKinematics
        // new PIDController(5.0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
        // new PIDController(5.0, 0, 0), // Y controller (usually the same values as X controller)
        // new PIDController(0.5, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
        // Drivetrain.getInstance()::setModuleStates, // Module states consumer
        // true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
        // Drivetrain.getInstance() // Requires this drive subsystem
        // );
        m_Drivetrain = Drivetrain.getInstance();
        
    }
    @Override
    public void initialize(){
        traj1 = PathPlanner.generatePath(
        new PathConstraints(2, 2), 
        new PathPoint(new Translation2d(1.0, 1.0), Rotation2d.fromDegrees(0)), // position, heading
        new PathPoint(new Translation2d(2.0, 1.0), Rotation2d.fromDegrees(0)) // position, heading
        );
        swerveCommand = new PPSwerveControllerCommand(traj1, m_Drivetrain::getPose, DrivetrainConstants.DRIVE_KINEMATICS, // SwerveDriveKinematics
        new PIDController(5.0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
        new PIDController(5.0, 0, 0), // Y controller (usually the same values as X controller)
        new PIDController(0.5, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
        m_Drivetrain::setModuleStates, // Module states consumer
        false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
        m_Drivetrain);
        swerveCommand.initialize();
    }
    @Override
    public void execute() {
        swerveCommand.execute();
    }
    @Override
    public boolean isFinished() {
        return swerveCommand.isFinished();
    }
    @Override
    public void end(boolean interrupted) {
        swerveCommand.end(interrupted);
    }
}
