package team3176.robot.constants;

public final class SwervePodConstants2022 extends DrivetrainHardwareMap {

    private static final double WHEEL_DIAMETER = DrivetrainConstants.WHEEL_DIAMETER;  // in inches
    //private static final double AZIMUTH_GEAR_RATIO = 70.0 / 1.0; // Is the Versa gearbox btwn motor & encoder
    private static final double AZIMUTH_GEAR_RATIO = 70.0 / 1.0; // Is the Versa gearbox btwn motor & encoder
    //private static final double AZIMUTH_GEAR_RATIO = 1.0 / 1.0; // Is the Versa gearbox btwn motor & encoder
    //private static final double THRUST_GEAR_RATIO = (54.0 / 14.0) * (48.0 / 30.0);  // 216/35?
    public static final double THRUST_GEAR_RATIO = (14.0/22.0) * (15.0/45.0);  

    
    
	/* Choose so that Talon does not report sensor out of phase */
	public static boolean[] SENSOR_PHASE_ = {false, false, false, false};

	/**
	 * Choose based on what direction you want to be positive,
	 * this does not affect motor invert. 
	 */
    public static boolean[] MOTOR_INVERTED_ = {false, false, false, false};



    public static final double AZIMUTH_ENCODER_UNITS_PER_REVOLUTION = 4096;
    public static final int AZIMUTH_PID_SLOT_ID = 0; 
    public static final int AZIMUTH_PID_LOOP_ID = 0; 
    public static final int AZIMUTH_ENCODER_TIMEOUT_MS = 0;  

    public static final double THRUST_ENCODER_UNITS_PER_REVOLUTION = 2048;
    public static final int[] TALON_THRUST_PID_SLOT_ID = {0, 0, 0, 0}; 
    public static final int[] TALON_THRUST_PID_LOOP_ID = {0, 0, 0, 0}; 
    public static final int[] TALON_THRUST_PID_TIMEOUT_MS = {0, 0, 0, 0};  
    
    public static final double[][] THRUST_PID_2022 = {
        /* kP */     {0.15, 0.15, 0.15, 0.15},
        /* kI */    {0.001, 0.001, 0.001, 0.001},
        /* kD */   {2.0, 2.0, 2.0, 2.0},
        /* kF */    {0.0065, 0.0065, 0.0065, 0.0065},    // Feed forward gain constant
        /* I-Zne */ {150.0, 150.0, 150.0, 150.0}     // The range of error for kI to take affect (like a reverse deadband)
    };

    public static double THRUST_PID[][] = THRUST_PID_2022;

    // BR P: 2.41, I: 0.0, D: 152.0, F: 0.0

    public static final double[][] AZIMUTH_PID_2022 = {

                {1.3, 1.3, 1.3, 1.3},
                {0.00,0.00,0.00,0.00},
                {.08, .08, .08, .08},
                {0.0, 0.0, 0.0, 0.0},    // Feed forward gain constant//<-Jared N550 internal encoder
                {0, 0.0, 0.0, 0.0},   //kIz constant//<-Jared N550 internal encoder
                {0.6000000238418579, 1.0, 1.0, 1.0},   //kMaxOutput//<-Jared N550 internal encoder
                {-0.6000000238418579, -1.0, -1.0, -1.0}  //kMinOutput//<-Jared N550 internal encoder
    };

    public static final double[] AZIMUTH_RAMPRATE = { 0.0, 0.0, 0.0, 0.0 };

    public static double AZIMUTH_PID[][] = AZIMUTH_PID_2022;


    /* OFFSETS: Corresponds to selftest output from CTRE Phoenix tool.
    *  Look for the absolute position encoder value.  Should say something 
    like:  "Pulsewidth/MagEnc(abs)"
    * Used solely for the AZIMUTH Encoder.
    */
   

    public static final double[] AZIMUTH_OFFSET_2022 = {0.0, 0.0, 0.0, 0.0}; 
    //public static final double[] AZIMUTH_OFFSET_2022 = {2703, 339, 2863, 2757}; // 2021 Bot //WTFOffsets
    public static final double[] AZIMUTH_OFFSET =AZIMUTH_OFFSET_2022;

    public static final double CHASSIS_SPEED_MAX_EMPIRICAL_FEET_PER_SECOND = 14.00;  //estimated loaded speed per Nathan;


    public static final double P_MODULE_THRUST_CONTROLLER = 1;
    public static final double[] P_MODULE_TURNING_CONTROLLER = {/*OLD P VALUES 1, 0, 0.3, 0.03*/ 0.08 /*kP*/, 0.0 /*kI*/, 0, 0.0};
    public static final double MAX_MODULE_ANGULAR_SPEED_RADIANS_PER_SECOND = 18000 * Math.PI;
    //public static final double MAX_MODULE_ANGULAR_SPEED_RADIANS_PER_SECOND = 1;
    public static final double MAX_MODULE_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 144000 * Math.PI;
    //public static final double MAX_MODULE_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 1;

    public static final double AZIMUTH_SPARKMAX_MAX_OUTPUTPERCENT = .50;

}

