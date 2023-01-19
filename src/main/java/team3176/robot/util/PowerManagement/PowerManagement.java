// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.util.PowerManagement;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.util.PowerManagement.ElectricalData;
import team3176.robot.constants.PowerConstants;


/** Add your docs here. */
public class PowerManagement extends SubsystemBase {
    private static PowerManagement instance = new PowerManagement();
    public static PowerManagement getInstance() {return instance;}

    //private int count = 0;
    private PowerDistribution powerDistributionHub;

    public PowerManagement() {
        // Change ModuleType to kCTRE if working with a CTRE PDP!!!!!
        powerDistributionHub = new PowerDistribution(1, ModuleType.kRev);
        // boolean compressor = true;
    }

    private ArrayList<Double> getPortData() {
        ArrayList<Double> kData = new ArrayList<Double>();
        // kData.add(powerDistributionHub.)

        return kData;
    }
    public void powerManagementPortZero() {
        double currentZero = powerDistributionHub.getCurrent(0);
    }

    @Override
    public void periodic() {
        int ports = powerDistributionHub.getNumChannels();
        /*
        count++;
          if(count > 100) {
           double battVolt = powerDistributionHub.getVoltage();     getBatteryVoltage
           if(battVolt < 10)
               disableCompressor
           if(battVolt > 10 && !compressor)
               enableCompressor
           count = 0;
          }
       */ 
    }

    public void clearStickyFaults()
    {
        powerDistributionHub.clearStickyFaults();
    }
    
}

/**

import frc.robot.constants.PowerManagementConstants;

public class PowerManagement extends SubsystemBase {
    private PowerDistributionPanel PDP; 
    private static PowerManagement instance = new PowerManagement();

    We assume an update rate of 1 update per 0.02 sec; therefore, we collect 5 values in the Current History,
     * which represents a window of 0.1 sec wide (10% of a second).
     * Consider expanding to 15 values which would represent a 0.25sec wide window.
   
    private ElectricalData HoodPM; 
    private ElectricalData DrumPM; 
   
    public PowerManagement() {
        PDP = new PowerDistributionPanel(PowerManagementConstants.PDP_CAN_ID);
        HoodPM = new ElectricalData(PowerManagementConstants.ANGLED_SHOOTER_PDP_CHANNEL, "Angled Shooter", 5);
        DrumPM = new ElectricalData(PowerManagementConstants.DRUM_PDP_CHANNEL, "Drum", 5);
    }

    public double getAngledShooterInstantAmp() {
        return HoodPM.getInstantaneousAmp();
    }

    public double getAngledShooterAvgAmp() {
        return HoodPM.mean();
    }

    public double getDrumInstantAmp() {
        return DrumPM.getInstantaneousAmp();
    }

    public double getDrumAvgAmp() {
        return DrumPM.mean();
    }

    @Override
    public void periodic() {
        HoodPM.addAmpData();
        DrumPM.addAmpData();
    }

    public void clearFaults() {
        PDP.clearStickyFaults();
    }

    public static PowerManagement getInstance() {
        return instance; 
    }

 * PDP CHANNEL MAP
 * System / Motor / Motor Controller / Encoder? / AmpFuse / CodeName / Slot#(Channel)
 * Swerve	Falcon	TalonFX	TalonFXInternal	40A	D1	1
 * Swerve	Falcon	TalonFX	TalonFXInternal	40A	D2	14
 * Swerve	Falcon	TalonFX	TalonFXInternal	40A	D3	15
 * Swerve	Falcon	TalonFX	TalonFXInternal	40A	D4	0
 * Swerve	775Pro	TalonSRX	Y- Versa	30A	G1	5
 * Swerve	775Pro	TalonSRX	Y- Versa	40A	G2	10
 * Swerve	775Pro	TalonSRX	Y- Versa	30A	G3	11
 * Swerve	775Pro	TalonSRX	Y- Versa	30A	G4	4
 * Intake	775Pro	TalonSRX	Y- Versa	30A	IN	6
 * Shooter	Falcon	N/A	N/A	40A	Shoot	12
 * Hood 	775Pro	TalonSRX	Y- Versa	30A	Angle	8
 * Drum NEO550	SparkMAX	N/A	40A	Drum	3
 * Transfer	775Pro	TalonSRX?	N/A	30A	Trans	9
 * Limelight	N/A	N/A	N/A	5A	LL	7
 * 
 
}
 */
