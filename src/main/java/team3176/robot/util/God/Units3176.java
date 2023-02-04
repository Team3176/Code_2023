package team3176.robot.util.God;

import edu.wpi.first.wpilibj.Timer;
import team3176.robot.*;
import team3176.robot.constants.DrivetrainConstants;
import team3176.robot.constants.SwervePodConstants2022;


public class Units3176{
	


	public double rads2Tics(double rads, double EncoderUnitsPerRevolution) {        
        //rads = rads * (2 * Math.PI);
        double rads_clamped = (Math.min((Math.max(rads,(-Math.PI))), (Math.PI)));        
        double tics = ((rads_clamped / (2.0*Math.PI)) * EncoderUnitsPerRevolution);
        return tics;
    }
	
	public int rads2Tics(double rads, double EncoderUnitsPerRevolution, boolean wantItInInts) {        
        //rads = rads * (2 * Math.PI);
        double rads_clamped = (Math.min((Math.max(rads,(-Math.PI))), (Math.PI)));        
        double tics = ((rads_clamped / (2.0*Math.PI)) * EncoderUnitsPerRevolution);
        return (int) tics;
    }

    public double tics2Rads(double tics, double EncoderUnitsPerRevolution) {
        tics = tics % EncoderUnitsPerRevolution;
        if(tics < 0) {
            tics += EncoderUnitsPerRevolution;
        }
        tics -= (EncoderUnitsPerRevolution / 2);
        return ((tics / EncoderUnitsPerRevolution) * (2 * Math.PI));
	}
    public static double conversion_feet_to_tics_per_100ms = 12.0 * (1.0/ (DrivetrainConstants.WHEEL_DIAMETER_INCHES * Math.PI)) * (1.0 /SwervePodConstants2022.THRUST_GEAR_RATIO) * SwervePodConstants2022.THRUST_ENCODER_UNITS_PER_REVOLUTION  * .1;
    public static double conversion_feet_to_tics = 12.0 * (1.0/ (DrivetrainConstants.WHEEL_DIAMETER_INCHES * Math.PI)) * (1.0 /SwervePodConstants2022.THRUST_GEAR_RATIO) * SwervePodConstants2022.THRUST_ENCODER_UNITS_PER_REVOLUTION;
    /**
      * Given feet per second, returns Falcon500 units (ie tics per 100ms)  
     * @param i feet per second
     * @return tics per 100ms
     */
    public static double fps2ums(double i) {
        
        // input * inchesPerFoot * circumfrenceOfWheel * ticsPerRev * gearRatio * secTo100ms
        return i * conversion_feet_to_tics_per_100ms;
        
        // return i * 100;
    }

    public static double ums2fps(double i) {
        return i / conversion_feet_to_tics_per_100ms;

    }
    /*
     * adding meters support for new WPILIB SI preference
     */
    public static double ums2mps(double i) {
        return ums2fps(i) * 0.3048;
    }
    public static double mps2ums(double i) {
        return fps2ums(i) * 3.2808;
    }


    public static double feetPerSecond2metersPerSecond(double i) {
        return i / 3.2808; 
    }
    
    public static double metersPerSecond2feetPerSecond(double i) {
        return i * 3.2808; 
    }
    
    public static double revolutionsPerMinute2ticsPer100MS(double maxRPM, double ticsPerRev) {
        return (maxRPM * ticsPerRev) / 600;
    }

}
