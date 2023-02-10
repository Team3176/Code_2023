package team3176.robot.constants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class ArmConstants {

    //Shoulder 
    public static final int SHOULDER_FALCON_CAN_ID = 6;
    public static final int SHOULDER_EXTENDED_LIMIT_CHAN = 1;
    public static final int SHOULDER_RETRACTED_LIMIT_CHAN = 2;

    //Elbow
    public static final int ELBOW_FALCON_CAN_ID = 5;
    public static final int ELBOW_FLOOR_LIMIT_CHAN = 3;
    public static final int ELBOW_PICKUP_LIMIT_CHAN = 4;

    //Elbow Discrete Positions
    //TODO: These values are just for testing purposes only
    public static final double PICKUP_POSITION = 0;
    public static final double HIGH_POSITION = 300;
    public static final double MID_POSITION = 600;
    public static final double FLOOR_POSITION = 900;

    //Wrist
    public static final int WRIST_FALCON_CAN_ID = 50;

    //PID 
    public static final int kPIDLoopIndex = 0;
    public static final int kTimeoutMS = 30;

    public static final double kRampRate = 0.5; // seconds from 0 to full speed...?

    public static final int TICKS_PER_REV = 4096;

    public static final int MAX_TICKSPER100MS = 2048; 

    //public static final String kShuffleboardPercentName1 = "Flywheel1%Set";

    //public static final String kShuffleboardPercentName1 = "ArmShoulder%Set";

    // Constant order: P, I, D, FF, IZone
    public static final double[][] Shoulder_PIDFConstants = { { 0.25, 0.0, 0.0, 0.0, 0.0 } };
    public static final double[][] Elbow_PIDFConstants = { { 0.25, 0.0, 0.0, 0.0, 0.0 } };
}
