package team3176.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.drivetrain.Drivetrain.driveMode;
public class AutoBalanceCenter extends CommandBase {
    private Drivetrain m_Drivetrain;
    private enum states {DRIVE,CORRECTING,WAITING,DONE};
    private states currentState;
    private double forward = 0.0;
    private double deadbandDegrees = 8;
    private Timer correctionTimer = new Timer();
    private Timer waitTimer = new Timer();
    public AutoBalanceCenter() {
        m_Drivetrain = Drivetrain.getInstance();
        addRequirements(m_Drivetrain);
    }
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        m_Drivetrain.setBrakeMode();
        currentState = states.DRIVE;
    }
    @Override
    public void execute() {
        //double Kp = 0.1;
        switch(currentState) {
            case CORRECTING:
            if(correctionTimer.get() < 0.5){
                m_Drivetrain.drive(-forward, 0, 0, Drivetrain.coordType.ROBOT_CENTRIC);
            }  else {
                currentState = states.WAITING;
                waitTimer.restart();
            }
                break;
            case DONE:
                break;
            case DRIVE:
            double forward = 0.0;
        
            if(m_Drivetrain.getChassisPitch() > 0 + deadbandDegrees) {
                forward = 0.39;
            }  else if(m_Drivetrain.getChassisPitch() < 0 - deadbandDegrees) {
                forward = -0.39;
            }  else {
                currentState = states.CORRECTING;
                correctionTimer.restart();
            }
            
            
            m_Drivetrain.drive(forward, 0, 0, Drivetrain.coordType.ROBOT_CENTRIC);
                break;
            case WAITING:
                if(waitTimer.get() < 0.5){
                    m_Drivetrain.drive(0, 0, 0, Drivetrain.coordType.ROBOT_CENTRIC);
                } else {
                    if(Math.abs(m_Drivetrain.getChassisPitch()) < deadbandDegrees) {
                        currentState = states.DONE;
                    }else {
                        currentState = states.DRIVE;
                    }
                
                }

                break;
            default:
                break;

        }

        
        //Bang Bang controller! 
        SmartDashboard.putNumber("pitch", m_Drivetrain.getChassisPitch());
    
    }
    @Override
    public void end(boolean interrupted) {
        m_Drivetrain.setDriveMode(driveMode.DEFENSE);
    }
    @Override
    public boolean isFinished() {
        return Timer.getMatchTime() < 0.5 || currentState == states.DONE;
    }

}
