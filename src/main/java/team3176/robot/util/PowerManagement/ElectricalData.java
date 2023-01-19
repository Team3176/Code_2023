package team3176.robot.util.PowerManagement;

import java.util.Arrays;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
//import team3176.robot.constants.PowerManagementConstants;
//import team3176.robot.subsystems.PowerManagement;

public class ElectricalData {
    String subsystemName = "UNKNOWN";  // Unique identifier that is readable and understandable to humans
    int pdpChannel;  //Location where subsystemName is plugged into PDP.  Should be a number ranging from 0 to ~15, typically.
    private double ampInstantaneous, ampMean, ampMedian, ampMode, ampRange; 

     /* Tracks the array-index of the ampHist array. Doing it this way, by using a 
     * persisting class variable to track the array, we can effectively make AmpHist array
     * a rolling window of amp values. Once windowTdx >= windowTdxMax, then 
     * windowTdx is reset to 0, and thus the next read amp value will be placed at index 0
     * of ampHist.  THEREFORE:  #1.) standard statical measurements (ie mean, median, mode, 
     * range, etc) are the best way to try and understand the meaning of data within ampHist
     * at any given moment in time, and #2.) the sequence of values in ampHist at any 
     * singular unique moment must be considered time-INdependent. */
    private int windowTdx, windowTdxMax;
    private double[] ampHist = {0.0};
    //PowerDistribution PDP = new PowerDistribution(PowerManagementConstants.PDP_CAN_ID, ModuleType.kCTRE);
    //PowerManagement m_PowerManagement;



    /**
     * <b> Electrical Data Structure <b>
     * Provides measurement and stats for a given channel on the PDP.
     * 
     * @param pdpChannel Channel on PDP from which to collect electrical data
     * @param subsystemName Name of Electical system you are measuring.  ie, the name of the mechanism, etc
     */
    public ElectricalData(int pdpChannel) {
        //m_PowerManagement = PowerManagement.getInstance();
        this.pdpChannel = pdpChannel;
        //initClassVars();
    }

    /**
     * <b> Electrical Data Structure <b>
     * Provides measurement and stats for a given channel on the PDP.
     * 
     * @param pdpChannel Channel on PDP from which to collect electrical data
     * @param subsystemName Name of Electical system you are measuring.  ie, the name of the mechanism, etc
     */
    public ElectricalData(int pdpChannel, String subsystemName) {
        this.pdpChannel = pdpChannel;
        this.subsystemName = subsystemName;
        //initClassVars();
    }

    /**
     * <b> Electrical Data Structure <b>
     * Provides measurement and stats for a given channel on the PDP.
     * 
     * @param pdpChannel Channel on PDP from which to collect electrical data
     * @param subsystemName Name of Electical system you are measuring.  ie, the name of the mechanism, etc 
     * @param statCollectionWindowSize Number of times to collect electrical data between statiscial analysis.  This defines the size of a rolling window upon which analysis is performed. <b> Default value = 5. <b> Data is collected approximately every 20ms.
     */
    public ElectricalData(int pdpChannel, String subsystemName, int statCollectionWindowSize) {
        this.pdpChannel = pdpChannel;
        this.subsystemName = subsystemName;
        //initClassVars(statCollectionWindowSize);
    }
    
    private void initClassVars(){
        windowTdx = 0;
        windowTdxMax = 5;
        resizeAmpHist(windowTdxMax);
    }
  
    private void initClassVars(int inputWindowSize){
        windowTdx = 0;
        windowTdxMax = inputWindowSize;
        resizeAmpHist(windowTdxMax);
    }

    private void resizeAmpHist(int windowTdxMax) {
        ampHist = Arrays.copyOf(ampHist, ampHist.length+(windowTdxMax - 1));
    }

   /* public double getInstantaneousAmp() {
        return m_PowerManagement.PDP.getCurrent(pdpChannel);
    }
    
    public void addAmpData() {
        if (windowTdx >= windowTdxMax) {windowTdx = 0;}
        ampHist[windowTdx] = getInstantaneousAmp();
        windowTdx += 1;
    }
    */

    public void addAmpData(double amp) {
        if (windowTdx >= windowTdxMax) {windowTdx = 0;}
        ampHist[windowTdx] = amp;
        windowTdx += 1;
    }


    public double sum() {
        double sum = 0;
        for(int idx=0; idx<windowTdxMax; idx++) {
            sum += ampHist[idx];
        }
        return sum;
    }

    public double mean() {
        return (sum() / windowTdxMax);
    }
    
    public double median() {
        double[] medianArray = ampHist;
        Arrays.sort(medianArray);
        double median;
        if (medianArray.length % 2 == 0) {
            median = ((double)medianArray[medianArray.length/2] + (double)medianArray[medianArray.length/2 - 1])/2;
        } else { 
            median = (double) medianArray[medianArray.length/2];
        }
        return median;
    }

    public double mode() {
        double maxValue = ampHist[0];
        int maxCount = 0;
        for (int idx = 0; idx < windowTdxMax; idx++) {
            int count = 0;
            for (int jdx = 0; jdx < windowTdxMax; jdx++) {
               if (ampHist[jdx] == ampHist[idx]){
                    count++;
                }
            }
            if (count > maxCount) {
                maxCount = count;
                maxValue = ampHist[idx];
            }
        }
        return maxValue;
    }

    public double min() {
        double min = ampHist[0]; 
        for (int idx = 1; idx < windowTdxMax; idx++) 
            min = Math.min(min, ampHist[idx]); 
        return min;
    }

    public double max() {
        double max = ampHist[0]; 
        for (int idx = 1; idx < windowTdxMax; idx++) 
            max = Math.max(max, ampHist[idx]); 
        return max;
    }

    public double range() {
        return (max()-min());
    }

    public double coeffOfRange() {
        return (range() / (max() + min()));
    }

    public double stdDev() {
        double m_mean = mean();
        double sqrdSum=0;
        for(int idx=0; idx<windowTdxMax; idx++) {
            sqrdSum += Math.pow((ampHist[idx]-m_mean),2);
        }
        m_mean = sqrdSum / (windowTdxMax - 1);
        double deviation = Math.sqrt(m_mean);
        return deviation;
    }
}
