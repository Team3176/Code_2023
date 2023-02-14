package team3176.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.drivetrain.Drivetrain;

public class AutoBalance extends CommandBase {
    private Drivetrain m_Drivetrain;
    
    public AutoBalance() {
        m_Drivetrain = Drivetrain.getInstance();
        addRequirements(m_Drivetrain);
    }
    @Override
    public void initialize() {
        // TODO Auto-generated method stub

    }
    @Override
    public void execute() {
        //double Kp = 0.1;

        //Bang Bang controller! 
        double forward = 0.0;
        double deadbandDegrees = 2;
        SmartDashboard.putNumber("pitch", m_Drivetrain.getChassisPitch());
        if(m_Drivetrain.getChassisPitch() > 0 + deadbandDegrees) {
            forward = 0.3;
        } else if(m_Drivetrain.getChassisPitch() < 0 - deadbandDegrees) {
            forward = -0.3;
        }
        m_Drivetrain.drive(forward, 0, 0, Drivetrain.coordType.ROBOT_CENTRIC);
    }


}
