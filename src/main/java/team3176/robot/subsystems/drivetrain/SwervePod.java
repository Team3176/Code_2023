package team3176.robot.subsystems.drivetrain;

import java.util.Map;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import team3176.robot.constants.SwervePodConstants2022;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import team3176.robot.util.God.*;


public class SwervePod {

    /** Class Object holding the Motor controller for Drive Motor on the SwervePod */
    private TalonFX thrustController;
    /** Class Object holding the Motor controller for azimuth (aka azimuth) Motor on the SwervePod */
    private CANSparkMax azimuthController;
    /** Class Object holding the CAN-based encoder for azimuth (aka azimuth) position of the SwervePod */
    CANCoder azimuthEncoder;
    /** Current value in radians of the azimuthEncoder's position */
    double azimuthEncoderRelPosition;
    double azimuthEncoderAbsPosition;
    boolean lastHasResetOccurred;

    /** Numerical identifier to differentiate between pods.
     *     For 4 Pods:  0 = FrontRight (FR),
     *                  1 = FrontLeft (FL),
     *                  2 = BackLeft (BL),
     *                  3 = BackRight (BR)
     */
    private int id;
    private String idString;
   
    /** Represents the value of the azimuthEncoder reading in radians when positioned with the positive Thrust vector of the Pod's Drive wheel pointing towards front of robot */
    //private double kAzimuthEncoderUnitsPerRevolution;

    private double lastEncoderPos;
    private double radianError;
    private double radianPos;
    private double encoderError;
    private double encoderPos;

    private double azimuthCommand;
    private double velTicsPer100ms;

    public int kSlotIdx_Azimuth, kPIDLoopIdx_Azimuth, kTimeoutMs_Azimuth,kSlotIdx_Thrust, kPIDLoopIdx_Thrust, kTimeoutMs_Thrust;

    public double podThrust, podAzimuth, podAbsAzimuth;

    private double kP_Azimuth;
    private double kI_Azimuth;
    private double kD_Azimuth;
    private double kRampRate_Azimuth = 0.0;

    private double kP_Thrust;
    private double kI_Thrust;
    private double kD_Thrust;
    private double kF_Thrust;

    private double turnOutput;
    private boolean isAutonSwerveControllerOptimizingAzimuthPos = false;

    private double PI = Math.PI;
    private final PIDController  m_turningPIDController2;
    //private final ProfiledPIDController m_turningProfiledPIDController;
    //private ProfiledPIDController m_turningPIDController;



    public SwervePod(int id, TalonFX thrustController, CANSparkMax azimuthController) {
        this.id = id;
        azimuthEncoder = new CANCoder(SwervePodConstants2022.STEER_CANCODER_CID[id]);
        azimuthEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        azimuthEncoder.configMagnetOffset(SwervePodConstants2022.AZIMUTH_ABS_ENCODER_OFFSET_POSITION[id]);
        azimuthEncoder.configSensorDirection(true,100);
        updateAzimuthAbsEncoder();
        initializeSmartDashboard();
        ///System.out.println("P"+(this.id+1)+" kEncoderOffset: "+this.kEncoderOffset);

        
        //kAzimuthEncoderUnitsPerRevolution = SwervePodConstants2022.AZIMUTH_ENCODER_UNITS_PER_REVOLUTION;
        kSlotIdx_Azimuth = SwervePodConstants2022.AZIMUTH_PID_SLOT_ID;
        kPIDLoopIdx_Azimuth = SwervePodConstants2022.AZIMUTH_PID_LOOP_ID;
        kTimeoutMs_Azimuth = SwervePodConstants2022.AZIMUTH_ENCODER_TIMEOUT_MS;
        
        kP_Thrust = 0.03; // SwervePodConstants.THRUST_PID[0][id];
        kI_Thrust = 0.0; // SwervePodConstants.THRUST_PID[1][id];
        kD_Thrust = 0.0; // SwervePodConstants.THRUST_PID[2][id];
        kF_Thrust = .045; // SwervePodConstants.THRUST_PID[3][id];


        this.kP_Azimuth = SwervePodConstants2022.AZIMUTH_PID[0][id];
        this.kI_Azimuth = SwervePodConstants2022.AZIMUTH_PID[1][id];
        this.kD_Azimuth = SwervePodConstants2022.AZIMUTH_PID[2][id];


        
      
        m_turningPIDController2 = new PIDController(kP_Azimuth, kI_Azimuth, kD_Azimuth);
        m_turningPIDController2.setTolerance(0.1);
        m_turningPIDController2.enableContinuousInput(-Math.PI, Math.PI);

        m_turningPIDController2.reset();
        m_turningPIDController2.setP(this.kP_Azimuth);
        m_turningPIDController2.setI(this.kI_Azimuth);
        m_turningPIDController2.setD(this.kD_Azimuth);
        

        //this.azimuthController.setOpenLoopRampRate(this.kRampRate_Azimuth);

        /**
		 * Config the allowable closed-loop error, Closed-Loop output will be
		 * neutral within this range. See Table in Section 17.2.1 for native
		 * units per rotation.
		 */
        //azimuthController.configAllowableClosedloopError(0, SwervePodConstants.kPIDLoopIdx, SwervePodConstants.kTimeoutMs);
        kSlotIdx_Thrust = SwervePodConstants2022.TALON_THRUST_PID_SLOT_ID[this.id];
        kPIDLoopIdx_Thrust = SwervePodConstants2022.TALON_THRUST_PID_LOOP_ID[this.id];
        kTimeoutMs_Thrust = SwervePodConstants2022.TALON_THRUST_PID_TIMEOUT_MS[this.id];



        //m_encoder = azimuthController.getEncoder();

        this.thrustController = thrustController;
        this.azimuthController = azimuthController;
        //this.AZIMUTHPIDController = azimuthController.getPIDController();

        this.thrustController.configFactoryDefault();
        //this.azimuthController.restoreFactoryDefaults();
        this.azimuthController.setOpenLoopRampRate(0.5);
        this.azimuthController.setInverted(true);
        //this.azimuthController.setClosedLoopRampRate(0.5);
        //this.azimuthController.burnFlash();

        this.thrustController.configClosedloopRamp(0.5);   
        
       
        this.thrustController.setInverted(false);
         
        //TODO: check out "Feedback Device Not Continuous"  under config tab in CTRE-tuner.  Is the available via API and set-able?  Caps encoder to range[-4096,4096], correct?
            //this.azimuthController.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition), 0, 0);
            //this.azimuthController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute), 0, 0);

        this.thrustController.config_kP(kPIDLoopIdx_Thrust, kP_Thrust, kTimeoutMs_Thrust);
        this.thrustController.config_kI(kPIDLoopIdx_Thrust, kI_Thrust, kTimeoutMs_Thrust);
        this.thrustController.config_kD(kPIDLoopIdx_Thrust, kD_Thrust, kTimeoutMs_Thrust);
        this.thrustController.config_kF(kPIDLoopIdx_Thrust, kF_Thrust, kTimeoutMs_Thrust);


        switch(id+1){ 
            case 1: this.idString="podFR";
                    break;
            case 2: this.idString="podFL";
                    break;
            case 3: this.idString="podBL";
                    break;
            case 4: this.idString="podBR";
                    break;
            default: this.idString="podNoExist";
                     break;
        }
    }




    /**
     *  alternative method for setting swervepod in line with WPILIB standard library
     * @param desiredState 
     */
    public void set_module(SwerveModuleState desiredState) {
        SwerveModuleState desired_optimized = SwerveModuleState.optimize(desiredState, Rotation2d.fromRadians(azimuthEncoderAbsPosition));
        if (this.podThrust > (-Math.pow(10,-10)) && this.podThrust < (Math.pow(10,-10))) {      
            this.turnOutput = m_turningPIDController2.calculate(this.azimuthEncoderAbsPosition, this.lastEncoderPos);
        } else {
            this.turnOutput = m_turningPIDController2.calculate(this.azimuthEncoderAbsPosition, desired_optimized.angle.getRadians());
            this.lastEncoderPos = desired_optimized.angle.getRadians(); 
        }
        azimuthController.set(turnOutput * SwervePodConstants2022.AZIMUTH_SPARKMAX_MAX_OUTPUTPERCENT);
        this.velTicsPer100ms = Units3176.fps2ums(desiredState.speedMetersPerSecond);
        thrustController.set(TalonFXControlMode.Velocity, velTicsPer100ms);
    }
    /**
     * @param podDrive (thrust) represents desired thrust of swervepod Range = -1 to 1 or
     *                 ft-per-sec?
     * @param podAzimuth  represents desired angle of swervepod. Range = -pi to pi.
     */
    public void set(double podDrive, double podAzimuth) {
        this.podThrust = podDrive;
        this.podAzimuth = podAzimuth; 
       
        this.velTicsPer100ms = Units3176.fps2ums(this.podThrust);
       // velTicsPer100ms = SmartDashboard.getNumber("thrustSet",velTicsPer100ms);
        double optmizdAzimuthAbsPos = optimizeAzimuthAbsPos(this.podAzimuth);
        //double optmizdAzimuthPos = this.podAzimuth;
        //double tics = rads2Tics(this.podAzimuth);


        if (this.podThrust > (-Math.pow(10,-10)) && this.podThrust < (Math.pow(10,-10))) {      
            this.turnOutput = m_turningPIDController2.calculate(this.azimuthEncoderAbsPosition, this.lastEncoderPos);
        } else {
            this.turnOutput = m_turningPIDController2.calculate(this.azimuthEncoderAbsPosition, optmizdAzimuthAbsPos);
            this.lastEncoderPos = optmizdAzimuthAbsPos; 
        }

        /*
        SmartDashboard.putNumber("P"+(id) + ".azimuthEncoderAbsPosition",this.azimuthEncoderAbsPosition); 
        SmartDashboard.putNumber("P"+(id) + ".optimizdAbsAzimuthPos",optmizdAzimuthAbsPos);
        SmartDashboard.putNumber("P"+(id) + ".turnOutput",turnOutput);
        */
 
        azimuthController.set(turnOutput * SwervePodConstants2022.AZIMUTH_SPARKMAX_MAX_OUTPUTPERCENT);
        thrustController.set(TalonFXControlMode.Velocity, velTicsPer100ms);
    }
    public SwerveModulePosition getPosition() {
        double fps = Units3176.ums2fps(thrustController.getSelectedSensorPosition());
        return new SwerveModulePosition(fps,Rotation2d.fromDegrees(azimuthEncoder.getAbsolutePosition()));
    }

    /**
     * @param angle desired angle of swerve pod in units of radians, range from -PI to +PI
     * @return
     */
    private double optimizeAzimuthAbsPos(double angle) {
        if (azimuthEncoder.hasResetOccurred() != this.lastHasResetOccurred) {
            this.lastHasResetOccurred = !this.lastHasResetOccurred;
            SmartDashboard.putBoolean("P"+this.id+".azimuthEncoderHasReset", this.lastHasResetOccurred); 
        }

        this.encoderPos = getEncoderAbsPos();
        radianPos = (this.encoderPos);
        SmartDashboard.putNumber("P"+(this.id)+".radianPos",radianPos);
        radianError = angle - radianPos;
        // SmartDashboard.putNumber("P"+(this.id)+".radianError_preOpt",radianError);
        // FYI: Math.copySign(magnitudeVar, signVar) = magnitude value with same sign as signvar

        // FIXES THE "SPINNIES" (wheels doing unnecessary rotations when starting and stopping)
        //int errorCorrectionIncrements = (int) (Math.abs(radianError) / (2 * Math.PI));
        //radianError -= Math.copySign((2 * Math.PI) * errorCorrectionIncrements, radianError);

        //if (Math.abs(radianError) > (5 * (PI / 2))) {
        //    System.out.println("Error: Overload");
        //} else if (Math.abs(radianError) > (3 * (PI / 2))) {

        if (isAutonSwerveControllerOptimizingAzimuthPos == false) { 
            if (Math.abs(radianError) > (3 * (PI / 2))) {      // TODO: See if commenting out "Thrust-vector sign-flip" fixes
                radianError -= Math.copySign(2 * PI, radianError);
            } else if (Math.abs(radianError) > (PI / 2)) {
                radianError -= Math.copySign(PI, radianError);
                this.velTicsPer100ms = -this.velTicsPer100ms;
            }
        }
        // SmartDashboard.putNumber("P"+(this.id)+".radianError_postOpt",radianError);
        //encoderError = rads2Tics(radianError);
        encoderError = (radianError);
        // SmartDashboard.putNumber("P"+(this.id)+".encoderError",encoderError);
        azimuthCommand = encoderError + this.encoderPos;
        // SmartDashboard.putNumber("P"+(this.id)+".AZIMUTHCmd",azimuthCommand);
        return (azimuthCommand);
    }


    public void goHome() {

        double optmizdAzimuthAbsPos = optimizeAzimuthAbsPos(0);

        // SmartDashboard.putNumber("P"+(this.id)+".optmizdazimuthAbsPos", optmizdAzimuthAbsPos);

        double turnOutput = m_turningPIDController2.calculate(this.azimuthEncoderAbsPosition, optmizdAzimuthAbsPos);
        //double turnOutput = m_turningPIDController.calculate(this.azimuthEncoderPosition, this.podAzimuth);

        // SmartDashboard.putNumber("P"+(this.id)+".turnOutput", turnOutput);
        azimuthController.set(turnOutput * SwervePodConstants2022.AZIMUTH_SPARKMAX_MAX_OUTPUTPERCENT);

        //this.azimuthPIDController.setReference(homePos, CANSparkMax.ControlType.kPosition);

    }

    public void setCurrentPositionAsHome() {
        this.azimuthEncoder.configMagnetOffset(0);
        double newAbsMagnetOffset = this.azimuthEncoder.getAbsolutePosition();
        this.azimuthEncoder.configMagnetOffset(-1 * newAbsMagnetOffset);
    }

   
    public boolean isInverted() { return azimuthController.getInverted(); }
    public void setInverted() { azimuthController.setInverted(!isInverted()); }

    public void updateAzimuthRelEncoder() {
        this.azimuthEncoderRelPosition = Math.toRadians(azimuthEncoder.getPosition());
        SmartDashboard.putNumber("P"+this.id+".azimuthEncoderPositionRad",Math.toRadians(azimuthEncoder.getPosition()));
        SmartDashboard.putNumber("P"+this.id+".azimuthEncoderPositionDeg",azimuthEncoder.getPosition());
    }

    public void updateAzimuthAbsEncoder() {
        this.azimuthEncoderAbsPosition = Math.toRadians(azimuthEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("P"+this.id+".azimuthEncoderAbsPositionRad",this.azimuthEncoderAbsPosition);
        SmartDashboard.putNumber("P"+this.id+".azimuthEncoderAbsPositionDeg",azimuthEncoder.getAbsolutePosition());
        // SmartDashboard.putNumber("P"+this.id+"azimuthEncoderCANOffset", SwervePodConstants2022.AZIMUTH_ABS_ENCODER_OFFSET_POSITION[id]);
        //NT_encoderPos.setDouble(this.azimuthEncoderAbsPosition);
    }

    public double getEncoderRelPos() {
        updateAzimuthRelEncoder();
        return this.azimuthEncoderRelPosition;
    }

    public double getEncoderAbsPos() {
        updateAzimuthAbsEncoder();
        return this.azimuthEncoderAbsPosition;
    } 
    
    public double getVelocity() {
        double motorShaftVelocity = thrustController.getSelectedSensorVelocity();   
        double wheelVelocityInFeetPerSecond = Units3176.ums2fps(motorShaftVelocity); 
        double wheelVelocityInMetersPerSecond = Units3176.feetPerSecond2metersPerSecond(wheelVelocityInFeetPerSecond);
        return wheelVelocityInMetersPerSecond;
    }

    /**
     * Returns current Azimuth of pod in degrees, where 0 is straight forward.
     * @return
     */
    public double getAzimuth() {
        return azimuthEncoder.getPosition(); 
    }

    public void boostThrustAcceleration() {
        this.thrustController.configClosedloopRamp(0.25);   
    }

    public void unboostThrustAcceleration() {
        this.thrustController.configClosedloopRamp(0.5);   
    }


    public void initializeSmartDashboard() {

        // SmartDashboard.putNumber("P"+(this.id)+".podAzimuth_setpoint_angle", 0);
        // SmartDashboard.putNumber("P"+(this.id)+".kP_Azimuth", 0);
        // SmartDashboard.putNumber("P"+(this.id)+".kI_Azimuth", 0);
        // SmartDashboard.putNumber("P"+(this.id)+".kD_Azimuth", 0);
        // SmartDashboard.putNumber("P"+(this.id)+".kRampRate_Azimuth", 0);
        SmartDashboard.putNumber("P"+(this.id)+".azimuthEncoderAbsPosition", this.azimuthEncoderAbsPosition);
        // SmartDashboard.putBoolean("P"+(this.id)+".On", false);
    }


    public void setupShuffleboard() {
        Shuffleboard.getTab(this.idString)
            .add(idString+"/podAzimuth_setpoint_angle",SwervePodConstants2022.AZIMUTH_ABS_ENCODER_OFFSET_POSITION[id])
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", -3.16, "max", 3.16))
            .withSize(2,1)
            .withPosition(2,1)
            .getEntry();
        Shuffleboard.getTab(this.idString)
            .add(idString+"/kP_Azimuth", this.kP_Azimuth)
            .withSize(1,1)
            .withPosition(4,1)
            .getEntry();
        Shuffleboard.getTab(this.idString)
            .add(idString+"/kI_Azimuth", this.kI_Azimuth)
            .withSize(1,1)
            .withPosition(5,1)
            .getEntry();
        Shuffleboard.getTab(this.idString)
            .add(idString+"/kD_Azimuth", this.kD_Azimuth)
            .withSize(1,1)
            .withPosition(6,1)
            .getEntry();




     
        //public void calculate() {
        //  double distance = ...;
        //  distanceEntry.setDouble(distance);
        //}
    }
    public void tune() {
        // SmartDashboard.getNumber("pt.kP_Azimuth", 0);
        // SmartDashboard.getNumber("pt.kI_Azimuth", 0);
        // SmartDashboard.getNumber("pt.kD_Azimuth", 0);
        //this.kP_Azimuth = NT_kP_Azimuth.getDouble(0);
        //this.kI_Azimuth = NT_kI_Azimuth.getDouble(0);
        //this.kD_Azimuth = NT_kD_Azimuth.getDouble(0);
        m_turningPIDController2.setP(this.kP_Azimuth);
        m_turningPIDController2.setI(this.kI_Azimuth);
        m_turningPIDController2.setD(this.kD_Azimuth);
        // SmartDashboard.putNumber("pt.kP_Azimuth", this.kP_Azimuth);
        // SmartDashboard.putNumber("pt.kI_Azimuth", this.kI_Azimuth);
        // SmartDashboard.putNumber("pt.kD_Azimuth", this.kD_Azimuth);
        //NT_kP_Azimuth.setDouble(this.kP_Azimuth);
        //NT_kI_Azimuth.setDouble(this.kI_Azimuth);
        //NT_kD_Azimuth.setDouble(this.kD_Azimuth);
        
        //SmartDashboard.putNumber("P"+(this.id)+".AzimuthAbsOffset", SwervePodConstants2022.AZIMUTH_ABS_ENCODER_OFFSET_POSITION[this.id]);
        // SmartDashboard.putNumber("pt.AzimuthAbsOffset", SwervePodConstants2022.AZIMUTH_ABS_ENCODER_OFFSET_POSITION[this.id]);
        // this.kRampRate_Azimuth = SmartDashboard.getNumber("pt.kRampRate_Azimuth", 0);

        this.azimuthController.setOpenLoopRampRate(this.kRampRate_Azimuth);
        //this.azimuthController.setSmartCurrentLimit(20);
        //this.azimuthController.burnFlash();

        //this.podAzimuth = SmartDashboard.getNumber("P"+(this.id)+".podAzimuth_setpoint_angle", SwervePodConstants2022.AZIMUTH_ABS_ENCODER_OFFSET_POSITION[this.id]);
        // this.podAzimuth = SmartDashboard.getNumber("pt.podAzimuth_setpoint_angle", 0);
        //this.podAzimuth = NT_podAzimuth_setpoint_angle.getDouble(0);

        //this.podAzimuth = 1;
        //System.out.println("podazimuth = "+this.podAzimuth);

        double optmizdAzimuthAbsPos = optimizeAzimuthAbsPos(this.podAzimuth);

        // SmartDashboard.putNumber("pt.optmizdazimuthPos", optmizdAzimuthAbsPos);
        // SmartDashboard.putNumber("pt.azimuthEncoderAbsPos", this.azimuthEncoderAbsPosition);
        SmartDashboard.putNumber("pt.azimuthEncoderAbsPosDeg", Math.toDegrees(this.azimuthEncoderAbsPosition));


        turnOutput = m_turningPIDController2.calculate(this.azimuthEncoderAbsPosition, optmizdAzimuthAbsPos);
        //double turnOutput = m_turningPIDController.calculate(this.azimuthEncoderPosition, this.podAzimuth);

        // SmartDashboard.putNumber("pt.turnOutput", turnOutput);
        azimuthController.set(turnOutput * SwervePodConstants2022.AZIMUTH_SPARKMAX_MAX_OUTPUTPERCENT);

    }
    public void SwervePod2022SmartDashboardComments () {
        //SwervePod2022 comments start
        // SmartDashboard.putNumber("P", kP_Thrust);
        // SmartDashboard.putNumber("I", kI_Thrust);
        // SmartDashboard.putNumber("D", kD_Thrust);
        // SmartDashboard.putNumber("F", kF_Thrust);
       // SmartDashboard.putNumber("driveSet",0);       
       
        // SmartDashboard.putNumber("P" + (id + 1) + " podDrive", this.podDrive);

        // SmartDashboard.putNumber("P" + (id + 1) + " podAzimuth", this.podAzimuth);
        //SwervePod2022 comments end


        //set(double podThrust, double podAzimuth) comments start
        // SmartDashboard.putNumber("P" + (id + 1) + " podThrust", this.podThrust);
        // SmartDashboard.putNumber("P" + (id + 1) + " podAzimuth", this.podAzimuth);

        // SmartDashboard.putNumber("fps2ums:velTicsPer100ms", velTicsPer100ms);
        // SmartDashboard.putNumber("podThrust", this.podThrust);

        // SmartDashboard.putNumber("P" + (id + 1) + " tics", tics);
        // SmartDashboard.putNumber("P" + (id + 1) + " absTics", azimuthController.getSelectedSensorPosition());

        // SmartDashboard.putNumber("P" + (id + 1) + " lastEncoderPos", this.lastEncoderPos);

        // SmartDashboard.putNumber("P" + (id + 1) + " lastEncoderPos", this.lastEncoderPos);

        //SmartDashboard.putNumber("P" + (id) + "getSelSenPos", azimuthController.getSelectedSensorPosition());

        // SmartDashboard.putNumber("P"+this.id+".podThrust", podThrust);
        //SmartDashboard.putNumber("actualVel", thrustController.getVoltage());
        
        // if (this.id == 0){
        //     SmartDashboard.putNumber("Pod3Distance", thrustController.getSelectedSensorPosition());
        //     SmartDashboard.putNumber("velTicsPer100ms", velTicsPer100ms);
        //     SmartDashboard.putNumber("Tics Error", thrustController.getSelectedSensorVelocity()-velTicsPer100ms);
        //     SmartDashboard.putNumber("Tics Error2", thrustController.getSelectedSensorVelocity()-velTicsPer100ms);
        //     SmartDashboard.putNumber("thrustController Velocity", thrustController.getSelectedSensorVelocity());
        // } 
        
        // SmartDashboard.putNumber("P" + (id + 1) + " velTicsPer100ms", velTicsPer100ms);
        // SmartDashboard.putNumber("P" + (id + 1) + " encoderSetPos_end", encoderSetPos);
        //}
        //set(double podthrust, double podAzimuth) comments start


        //private double optimizeAzimuthPos(double angle) comments start
        // SmartDashboard.putNumber("P" + (id + 1) + " calcAzimuthPos_angle", angle);
        
        // SmartDashboard.putNumber("P" + (id + 1) + " kEncoderOffset", this.kEncoderOffset);
        // SmartDashboard.putNumber("P" + (id + 1) + " getSelectedSensorPosition", azimuthController.getSelectedSensorPosition());
        // SmartDashboard.putNumber("P" + (id + 1) + " encoderPos_in_calcAzimuthPos",this.encoderPos);

        // SmartDashboard.putNumber("P" + (id + 1) + " radianPos", radianPos);

        // SmartDashboard.putNumber("P" + (id + 1) + " radianError", radianError);

        // SmartDashboard.putNumber("P" + (id + 1) + " encoderError", encoderError);

        // SmartDashboard.putNumber("P" + (id + 1) + "tics2radianThrustcommand", thrustCommand);
        //private double optimizeAzimuthPos(double angle) comments end


        //public void setDesiredState(SwerveModuleState desiredState) comments start
        // SmartDashboard.putNumber("P"+this.id+".setDesiredState_desiredDegrees", desiredState.angle.getDegrees());
        // SmartDashboard.putNumber("P"+this.id+".setdesiredState_sensoredDegrees", rotation.getDegrees());

        // SmartDashboard.putNumber("P"+this.id+".setDesiredState_TurnOutput",turnOutput);
        // SmartDashboard.putNumber("P"+this.id+".setDesiredState_ThrustOutput", thrustOutput);
        //public void setDesiredState(SwerveModuleState desiredState) comments ends


        //public double getVelocity_metersPerSec() comments start
        //SmartDashboard.putNumber("GetSensorVelocity", speed);
        // SmartDashboard.putNumber("Velocity", metersPerSecond);
        //public double getVelocity_metersPerSec() comments end
        
    }

 // public double getRate(){
 // }
}
