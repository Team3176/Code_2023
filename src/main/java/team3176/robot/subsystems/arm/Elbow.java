package team3176.robot.subsystems.arm;

import java.beans.Encoder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.constants.ArmConstants;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import team3176.robot.subsystems.arm.ElbowIO;
import team3176.robot.subsystems.arm.ElbowIO.ElbowIOInputs;
import org.littletonrobotics.junction.Logger;

public class Elbow extends SubsystemBase {

private static Elbow instance;
private final ElbowIO io;
private final ElbowIOInputs inputs = new ElbowIOInputs();

private WPI_TalonFX elbowMotor;
private ElbowState currentState;
private String currentStateString;
private CANCoder elbowCANCoder;

/* Discrete positions for the arm */
/* This will likely need to be defined elsewhere so that callers have access to this type
 * in order to command the arm to the desired state.
 */
public enum ElbowState {
    UNKNOWN,
    PICKUP,
    HIGH,
    MID,
    FLOOR,
    TRANSITIONING   //Optional state that is currently not used
}

public Elbow(ElbowIO io) {
    this.io = io;
    elbowMotor = new WPI_TalonFX (ArmConstants.ELBOW_FALCON_MOTOR_CID, "rio");
	elbowCANCoder = new CANCoder(ArmConstants.ELBOW_CANCODER_CID,"rio");
	SlewRateLimiter filterLeftYStick = new SlewRateLimiter(0.1);
	SlewRateLimiter filterRightYStick = new SlewRateLimiter(0.1);

	/* Used to build string throughout loop */
	StringBuilder _sb = new StringBuilder();

	/** How much smoothing [0,8] to use during MotionMagic */
	//int _smoothing = (team3176.robot.constants.(team3176.robot.constants.(team3176.robot.constants.(team3176.robot.constants.(team3176.robot.constants.ArmConstants.ELBOW_MM_SMOOTHING;

    currentState = ElbowState.UNKNOWN;
    currentStateString = "Unknown";
    robotInit();

    System.out.println("Elbow Motor has been constructed");

   
}
public void robotInit() {
    /* Factory default hardware to prevent unexpected behavior */
    elbowMotor.configFactoryDefault();

    /* Configure CANCoder as an absolute position encoder that returns degrees.
    The encoder is intended to provide the absolute position when the robot is
    first powered on. This allows the arm to be in any position on power-up.
    Subsequent control of the arm rotation will use the integrated encoder in
    the TalonFX via Motion Magic.
    */
    elbowCANCoder.configFactoryDefault();

    //TODO: Understand the timeout value
    elbowCANCoder.configSensorDirection(false, 100); //positive rotation is CCW when facing LED side of encoder
    //armEncoder.configSensorDirection(true, 100); //positive rotation is CW when facing LED side of encoder
    
    elbowCANCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    //TODO: Set the offset so that the CANCoder reports 0 degrees at the bottom of the rotation.
    //Note that this is potentially OUTSIDE the functional range of the arm
    elbowCANCoder.configMagnetOffset(ArmConstants.ELBOW_CANCODER_MAGNET_OFFSET_DEG);

    //armEncoder.configGetFeedbackCoefficient(0.087890625,"deg", SensorTimeBase.PerSecond);
    //armEncoder.setPositionToAbsolute();
    SmartDashboard.putNumber("ARM Abs Pos (deg)", elbowCANCoder.getAbsolutePosition());


    elbowMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, ArmConstants.kPIDLoopIndex,	ArmConstants.kTimeoutMS);

    /* set deadband to super small 0.001 (0.1 %).
        The default deadband is 0.04 (4 %) */
    elbowMotor.configNeutralDeadband(0.001, ArmConstants.kTimeoutMS);

    /**
     * Configure Talon FX Output and Sensor direction accordingly Invert Motor to
     * have green LEDs when driving Talon Forward / Requesting Postiive Output Phase
     * sensor to have positive increment when driving Talon Forward (Green LED)
     */
    elbowMotor.setSensorPhase(false);
    elbowMotor.setInverted(false);
    /*
     * Talon FX does not need sensor phase set for its integrated sensor
     * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
     * and the user calls getSelectedSensor* to get the sensor's position/velocity.
     * 
     * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
     */
    // elbowMotor.setSensorPhase(true);

    /* Set relevant frame periods to be at least as fast as periodic rate */
    elbowMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, ArmConstants.kTimeoutMS);
    elbowMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, ArmConstants.kTimeoutMS);

    /* Set the peak and nominal outputs */
    elbowMotor.configNominalOutputForward(0, ArmConstants.kTimeoutMS);
    elbowMotor.configNominalOutputReverse(0, ArmConstants.kTimeoutMS);
    elbowMotor.configPeakOutputForward(1, ArmConstants.kTimeoutMS);
    elbowMotor.configPeakOutputReverse(-1, ArmConstants.kTimeoutMS);

    /* Set Motion Magic gains in slot0 - see documentation */
    elbowMotor.selectProfileSlot(ArmConstants.kSlotIdx,ArmConstants.kPIDLoopIndex);
    elbowMotor.config_kF(ArmConstants.kSlotIdx, ArmConstants.Elbow_PIDFConstants[3], ArmConstants.kTimeoutMS);
    elbowMotor.config_kP(ArmConstants.kSlotIdx, ArmConstants.Elbow_PIDFConstants[0], ArmConstants.kTimeoutMS);
    elbowMotor.config_kI(ArmConstants.kSlotIdx, ArmConstants.Elbow_PIDFConstants[1], ArmConstants.kTimeoutMS);
    elbowMotor.config_kD(ArmConstants.kSlotIdx, ArmConstants.Elbow_PIDFConstants[2], ArmConstants.kTimeoutMS);
    /* Set acceleration and vcruise velocity - see documentation */
    elbowMotor.configMotionCruiseVelocity(ArmConstants.ELBOW_MM_CRUISE_VELOCITY, ArmConstants.kTimeoutMS);
    elbowMotor.configMotionAcceleration(ArmConstants.ELBOW_MM_ACCELERATION, ArmConstants.kTimeoutMS);

    //Initialize the arm position on startup
    initArmPosition();
}


public void initArmPosition() {

    /* Set the integrated sensor to the arm position. Translate from the CANCoder
    degrees to ticks in the Talon. Execute once on robot boot up and any time
    that it could be out of sync (like when enabling the robot)
     */
    double initialTickPosition = ArmConstants.ELBOW_TICKS_PER_DEG_ROTATION
                                        * ArmConstants.ELBOW_MOTOR_TO_ROTATING_SHAFT_RATIO
                                        * elbowCANCoder.getAbsolutePosition();
    elbowMotor.setSelectedSensorPosition(initialTickPosition, ArmConstants.kPIDLoopIndex, ArmConstants.kTimeoutMS);
    System.out.println("initialTickPosition:"+initialTickPosition);
}

/* Set the desired discrete position of the elbow */
public void setElbowState(ElbowState targetState ) {

    switch (targetState) {
        case PICKUP:
//            elbowMotor.set(ControlMode.Position, (team3176.robot.constants.ArmConstants.ELBOW_PICKUP_POSITION_DEG));
            currentState = targetState;
            currentStateString = "Pickup";
            break;
       
        case HIGH:
  //          elbowMotor.set(ControlMode.Position,(team3176.robot.constants.ArmConstants.ELBOW_HIGH_POSITION_DEG));
            currentState = targetState;
            currentStateString = "High";
            break;

        case MID:
//            elbowMotor.set(ControlMode.Position, (team3176.robot.constants.ArmConstants.ELBOW_MID_POSITION_DEG));
            currentState= targetState;
            currentStateString = "Mid";
            break;

        case FLOOR:
 //           elbowMotor.set(ControlMode.Position, (team3176.robot.constants.ArmConstants.ELBOW_FLOOR_POSITION_DEG));
            currentState = targetState;
            currentStateString = "Floor";
            break;

        default:
            currentStateString = "Invalid";
            break; 
    
    }

    SmartDashboard.putString("Elbow Position", currentStateString);  

}

public ElbowState getElbowState() {

    return currentState;
}

/* Returns the position of the elbow */
public double getCurrentPosition() {
    return elbowMotor.getSelectedSensorPosition();

}

public static Elbow getInstance() {
    if(instance == null) {instance = new Elbow(new ElbowIO() {});}
    return instance;
  }
  public boolean clawIsHoldingCone() {
    //TODO: determine a way to determine if the claw is holding a cone
   return false;
}

   /* If the robot is holding a cone we will want to apply the necessary arbitrary feed 
    * forward offset to maintain the desired position. A claw that is empty or
    * holding a cube doesn't require compensation. For the rotating arm use the angle to
    * determine the correct offset for gravity. Characterize the max value with the arm
    * in the horizontal position.
    * 
    * 
    * 
    */
   public double getArbFeedForward() {

       double arbitraryFF = 0.0;

       if (clawIsHoldingCone()) {
             //motor sensor position measured when arm is horizontal 
            double kMeasuredPosHorizontal = ArmConstants.ELBOW_TICKS_PER_DEG_ROTATION * ArmConstants.ELBOW_MOTOR_TO_ROTATING_SHAFT_RATIO * 270.0; 
           double currentPos = elbowMotor.getSelectedSensorPosition();
           double degrees = (currentPos - kMeasuredPosHorizontal) / 
                                   ArmConstants.ELBOW_TICKS_PER_DEG_ROTATION * ArmConstants.ELBOW_MOTOR_TO_ROTATING_SHAFT_RATIO;
           double radians = java.lang.Math.toRadians(degrees);
           double cosineScalar = java.lang.Math.cos(radians);

           arbitraryFF = ArmConstants.ELBOW_MAX_GRAVITY_FF * cosineScalar;
       //_motorcontroller.set(ControlMode.MotionMagic, targetPos, DemandType.ArbitraryFeedForward, arbitraryFeedForward);
       }	

       return arbitraryFF;

   
   }



/* The target position will be checked for out of range using the specified soft limits
*/
   public void setPosition(double reqTargetDegrees) {

    StringBuffer _sb = new StringBuffer();
    double motorOutput = elbowMotor.getMotorOutputPercent();

    /* Prepare line to print */
		_sb.append("\tOut%:");
		_sb.append(motorOutput);
		_sb.append("\tVel:");
		_sb.append(elbowMotor.getSelectedSensorVelocity(ArmConstants.kPIDLoopIndex));

       if (reqTargetDegrees < ArmConstants.ELBOW_SOFT_LIMIT_PICKUP_DEG) {
           SmartDashboard.putString("ARM Req Deg", ArmConstants.ELBOW_SOFT_LIMIT_PICKUP_DEG+"("+reqTargetDegrees+")");
           System.out.println("Requested degrees ("+reqTargetDegrees + ") exceeds soft limit of "+ArmConstants.ELBOW_SOFT_LIMIT_PICKUP_DEG+"degrees");
           reqTargetDegrees = ArmConstants.ELBOW_SOFT_LIMIT_PICKUP_DEG;

       }
       else if (reqTargetDegrees > ArmConstants.ELBOW_SOFT_LIMIT_FLOOR_DEG) {
           SmartDashboard.putString("ARM Req Deg", ArmConstants.ELBOW_SOFT_LIMIT_FLOOR_DEG+"("+reqTargetDegrees+")");
           System.out.println("Requested degrees ("+reqTargetDegrees + ") exceeds soft limit of "+ArmConstants.ELBOW_SOFT_LIMIT_FLOOR_DEG+"degrees");
           reqTargetDegrees = ArmConstants.ELBOW_SOFT_LIMIT_FLOOR_DEG;
       }
       else {
           SmartDashboard.putNumber("ARM Req Deg", reqTargetDegrees);
       }

       double actTargetPos = reqTargetDegrees 
                               * ArmConstants.ELBOW_TICKS_PER_DEG_ROTATION 
                               * ArmConstants.ELBOW_MOTOR_TO_ROTATING_SHAFT_RATIO;

       elbowMotor.set(TalonFXControlMode.MotionMagic, actTargetPos, DemandType.ArbitraryFeedForward, getArbFeedForward());
    	/* Append more signals to print when in speed mode */
		_sb.append("\terr:");
		_sb.append(elbowMotor.getClosedLoopError(ArmConstants.kPIDLoopIndex));
		_sb.append("\ttrg:");
		_sb.append(actTargetPos);

   }

   @Override
   public void periodic() 
   {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Elbow", inputs);
    Logger.getInstance().recordOutput("Elbow/Velocity", getVelocity());
    Logger.getInstance().recordOutput("Elbow/Position", getPosition());
   }

   public double getVelocity()
   {
    return inputs.velocity;
   }

   public double getPosition()
   {
    return inputs.position;
   }

   public void runVoltage(double volts)
   {
    io.setVoltage(volts);
   }

   public void setVelocity(double velocity)
   {
    io.setVelocity(velocity);
   }

   public void setPositionLogger(double position)
   {
    io.setPosition(position);
   }
}