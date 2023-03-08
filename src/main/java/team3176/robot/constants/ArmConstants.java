package team3176.robot.constants;

import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Topic;

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

    //TODO: This file is for Constants only. Move non-constants to the appropriate classes
    public static int wasLimitSwitchPressed;
    public static int SHOULDER_POSITION;

    public static int PidTime;

    //Wrist
    public static final int WRIST_MOTOR_CID = 50;

    //PID 
    public static final int kSlotIdx = 0;
    public static final int kPIDLoopIndex = 0;
    public static final int kTimeoutMS = 30;

    public static final double kRampRate = 0.5; // seconds from 0 to full speed...?

    public static final int TICKS_PER_REV = 4096;
    public static final int MAX_TICKSPER100MS = 2048; 

    //public static final String kShuffleboardPercentName1 = "Flywheel1%Set";

    //public static final String kShuffleboardPercentName1 = "ArmShoulder%Set";

    // Constant order: P, I, D, FF, IZone
    public static final double[][] Shoulder_PIDFConstants = { { 0.1, 0.0025, 0.0, 0.0, 1 } };
    
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("datatable");

    Topic genericTopic = inst.getTopic("/datatable/X");
    DoubleTopic dblTopic = new DoubleTopic(genericTopic);


    //ELBOW Constants
    public static final int ELBOW_FALCON_MOTOR_CID = 52;
    public static final int ELBOW_CANCODER_CID = 11;
	public static final int ELBOW_PICKUP_LIMIT_CHAN = 3;
	public static final int ELBOW_FLOOR_LIMIT_CHAN = 4;

	//Elbow Discrete Positions
    //TODO: These values are just for testing purposes only
    public static final double ELBOW_PICKUP_POSITION_DEG = 30;
    public static final double ELBOW_HIGH_POSITION_DEG = 120;
    public static final double ELBOW_MID_POSITION_DEG = 180;
    public static final double ELBOW_FLOOR_POSITION_DEG = 330;

    //Encoder Ticks/360degrees
    public static final double ELBOW_TICKS_PER_DEG_ROTATION = 11.37778; 	// 4096/360
    public static final double ELBOW_MOTOR_TO_ROTATING_SHAFT_RATIO = 0.7; //1.42;   // 38/54
    public static final double ELBOW_CANCODER_MAGNET_OFFSET_DEG = -324.0;   //Test fixture with 0deg at the bottom

    public static final double ELBOW_MM_FEED_FORWARD = 0.04716;
	public static final double ELBOW_MM_CRUISE_VELOCITY = 2000;
	public static final double ELBOW_MM_ACCELERATION = 3000;
	public static final int    ELBOW_MM_SMOOTHING = 0;

    public static final double ELBOW_MAX_GRAVITY_FF = 0.07;
    
    public static final double ELBOW_SOFT_LIMIT_PICKUP_DEG = 20;  
	public static final double ELBOW_SOFT_LIMIT_FLOOR_DEG = 340; 

    /**
	 * Gains used in Motion Magic, to be adjusted accordingly
     * Gains(kp, ki, kd, kf, izone, peak output);
     */
    //static final Gains kGains = new Gains(0.069, 0.0, 0.0, ELBOW_MM_FEED_FORWARD, 0, 1.0);

    // Constant order: P, I, D, FF, IZone
    public static final double[] Elbow_PIDFConstants = { 0.125, 0.0, 0.0, ELBOW_MM_FEED_FORWARD, 0.1 };


}