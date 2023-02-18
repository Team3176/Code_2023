package team3176.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.drivetrain.Drivetrain;

public class AutoBalance extends CommandBase {
    private Drivetrain m_Drivetrain;
    private boolean isDone = false;
    private int num_balanced = 0;
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
        double deadbandDegrees = 8;
        SmartDashboard.putNumber("pitch", m_Drivetrain.getChassisPitch());
        if(m_Drivetrain.getChassisPitch() > 0 + deadbandDegrees) {
            forward = 0.37 * Math.pow(.97,num_balanced);
        } else if(m_Drivetrain.getChassisPitch() < 0 - deadbandDegrees) {
            forward = -0.37 * Math.pow(.97,num_balanced);
        } else {
            num_balanced ++;
        }
        m_Drivetrain.drive(forward, 0, 0, Drivetrain.coordType.ROBOT_CENTRIC);
    }
    @Override
    public boolean isFinished() {
        return num_balanced>20;
    }

}
