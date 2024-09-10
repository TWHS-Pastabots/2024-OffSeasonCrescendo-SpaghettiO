package frc.robot.subsystems.launcher;

import java.util.HashMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.LauncherConstants;
import frc.robot.subsystems.IO.DigitalInputs;
//import frc.robot.subsystems.vision.VisionTablesListener;
import frc.robot.Ports;

public class Launcher {
    //launcherstates include the launch speed along with the rotations for the launcher motor 
    public enum LauncherState {
        // AMP(-48.5, 1.0),
        AMP(-60.5, 0.8),
        ALTAMP(-55, 0.9),
        START(0, 0.0),
        TRAP(-70.04991149902344, 0.8),
        LONG(-15.75, 1.0),
        //was 9 before (HANDOFF)
        HANDOFF(8.5, 0.5),
        //was -3 before (HOVER)
        HOVER(-4, 1.0),
        TOSS(-22, .80),
        AUTOMIDSHOT(-12, 1.0),
        // height: ?
        AUTOLEFTSHOT(-13.5, 1.0),
        // height: 20.75    
        AUTORIGHTSHOT(-13.5, 1.0),
        // height: ?7
        //angle from the negative vertical (3pi/2 to pi type beat): 149.85
        SPEAKER(-54, 1.0),
        
        ALTSPEAKER(-23, 1.0),
        INTERLOPE(0.0, 1.0),
        TEST(-13.25, 1.0),
        FIXER(-20, 0);

        public double position;
        public double launchSpeed;
        
        private LauncherState(double position, double launchSpeed) {
            this.position = position;
            this.launchSpeed = launchSpeed;
        }
    }
    //the same as above but for the amp mechanism
    public enum AmpMotorPos {
        DOWN(-0.25),
        UP(-20);

        public double position;

        private AmpMotorPos(double position) {
            this.position = position;
        }
    }
    //getting instances of every motor 
    double anglePower = 0.2;

    private CANSparkMax shootMotor1;
    private CANSparkMax shootMotor2;

    private CANSparkMax flicker;

    private CANSparkMax pivotMotor;

    private CANSparkMax ampMotor;

    private double increment = 1.0;

    private ArmFeedforward feedForward;
    private ArmFeedforward ampMotorFeedForward;

    private SparkMaxPIDController pivotController1;

    private SparkMaxPIDController ampMotorController;

    private static RelativeEncoder encoder;

    private static RelativeEncoder boxScore;

    private DigitalInputs breakBeam;

    private boolean[] connections = new boolean[8];

    private static LauncherState launchState = LauncherState.START;
    private static AmpMotorPos ampPose= AmpMotorPos.DOWN;

    public static Launcher instance;

    //public VisionTablesListener visTables;

    // private HashMap<Double, Double> lookupTable = new HashMap<>();
    // private double[] bluePositions = new double[] { 1.62, 1.93, 2.34, 2.41, 2.63, 2.71, 2.94, 3.01, 3.3, 4.14 };

    public Launcher() {
        shootMotor1 = new CANSparkMax(Ports.shootMotor1, MotorType.kBrushless);
        shootMotor1.restoreFactoryDefaults();

        shootMotor1.setSmartCurrentLimit(60);
        shootMotor1.setIdleMode(IdleMode.kBrake);
        shootMotor1.setInverted(false);
        shootMotor1.burnFlash();

        shootMotor2 = new CANSparkMax(Ports.shootMotor2, MotorType.kBrushless);
        shootMotor2.restoreFactoryDefaults();

        shootMotor2.setSmartCurrentLimit(60);
        shootMotor2.setIdleMode(IdleMode.kBrake);
        shootMotor2.setInverted(false);
        shootMotor2.burnFlash();

        flicker = new CANSparkMax(Ports.flicker, MotorType.kBrushless);
        flicker.restoreFactoryDefaults();

        flicker.setSmartCurrentLimit(20);
        flicker.setIdleMode(IdleMode.kBrake);
        flicker.setInverted(false);
        flicker.burnFlash();

        pivotMotor = new CANSparkMax(Ports.pivotMotor, MotorType.kBrushless);
        pivotMotor.restoreFactoryDefaults();

        pivotMotor.setSmartCurrentLimit(60);
        pivotMotor.setIdleMode(IdleMode.kBrake);
        pivotMotor.setInverted(true);
        // pivotMotor.setInverted(false);

        pivotMotor.setOpenLoopRampRate(1);

        ampMotor = new CANSparkMax(Ports.lebron, MotorType.kBrushless);
        ampMotor.restoreFactoryDefaults();

        ampMotor.setSmartCurrentLimit(20);
        ampMotor.setIdleMode(IdleMode.kBrake);

        feedForward = new ArmFeedforward(0.012, 0.017, 0.0, 0.0);
        // u:.023 l:.011 mid:.017 ks:.012

        ampMotorFeedForward = new ArmFeedforward(0, 0, 0);

        encoder = pivotMotor.getEncoder();

        pivotController1 = pivotMotor.getPIDController();

        pivotController1.setP(LauncherConstants.pivotPCoefficient);
        pivotController1.setI(LauncherConstants.pivotICoefficient);
        pivotController1.setD(LauncherConstants.pivotDCoefficient);

        pivotController1.setFeedbackDevice(encoder);

        pivotController1.setOutputRange(-1, 1);

        ampMotorController = ampMotor.getPIDController();

        boxScore = ampMotor.getEncoder();
        boxScore.setPositionConversionFactor(1);

        ampMotorController.setFeedbackDevice(boxScore);

        ampMotorController.setOutputRange(-1, 1);

        ampMotorController.setP(LauncherConstants.lebronPCoefficient);
        ampMotorController.setI(LauncherConstants.lebronICoefficient);
        ampMotorController.setD(LauncherConstants.lebronDCoefficient);

        pivotMotor.burnFlash();
        ampMotor.burnFlash();

        breakBeam = DigitalInputs.getInstance();

        

    }

    public void updatePose() {

        pivotController1.setReference(launchState.position, CANSparkMax.ControlType.kPosition, 0,
                feedForward.calculate(encoder.getPosition(), 0));

        

    }

    public void moveAmp() {
        ampMotorController.setReference(ampPose.position, CANSparkMax.ControlType.kPosition, 0,
                ampMotorFeedForward.calculate(boxScore.getPosition(), 0));
    }

    public void interpolateAngle() {
        double deltaX;

    
     }

    public void eject() {
        shootMotor2.set(0);
        shootMotor1.set(launchState.launchSpeed);
    }

    public void ampOn() {
        ampMotor.set(-0.5);
    }

    public void ampReverse() {
        ampMotor.set(0.5);
    }

    public void ampOff() {
        ampMotor.set(0.0);
    }

    public void setPivotOff() {
        pivotMotor.set(0.0);
    }

    public double getTestPosition() {
        return LauncherState.TEST.position;
    }

     public double getSpeakerPosition() {
        return LauncherState.SPEAKER.position;
    }

    public double getAmpPostion() {
        return boxScore.getPosition();
    }

    public double getAmpCurrent(){
        return ampMotor.getOutputCurrent();

    }

    

    

    public void setLauncherOn() {
        if (launchState == LauncherState.AMP) {
           
            shootMotor1.set(launchState.launchSpeed * 0.1);
            shootMotor2.set(launchState.launchSpeed * 0.1);
        } 
        else {
            shootMotor1.set(launchState.launchSpeed);
            shootMotor2.set(launchState.launchSpeed);
        }
    }

    public void setReverseLauncherOn() {

        
        shootMotor1.set(-launchState.launchSpeed);
        shootMotor2.set(-launchState.launchSpeed);
        
    }

    public void setLauncherOff() {
        shootMotor1.set(0.0);
        shootMotor2.set(0.0);
    }

    public void setFlickerOn() {
        flicker.set(1.0);
    }

    public void setFlickerReverse() {
        flicker.set(-1.0);
    }

    public void 
    setFlickerPartial() {
        flicker.set(0.85);
    }

    public void setFlickOff() {
        flicker.set(0);
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    public boolean getBreakBeam() {
        return !breakBeam.getInputs()[Ports.launcherBreakBeam];
    }

    public LauncherState getLaunchState() {
        return launchState;
    }

    public double getPivotCurrent() {
        return pivotMotor.getOutputCurrent();
    }

    public boolean hasReachedPose(double tolerance) {
        return Math.abs(getPosition() - launchState.position) < tolerance;
    }

    public void setLauncherState(LauncherState state) {
        launchState = state;
    }

    public void setAmpPose(AmpMotorPos position) {
        ampPose = position;
    }

    public void increaseIncrement() {
        increment += 0.25;
    }

    public void decreaseInrement() {
        increment -= 0.25;
    }

    public void increasePosition() {
        
        if (launchState == LauncherState.SPEAKER) {
            LauncherState.SPEAKER.position = LauncherState.SPEAKER.position + increment;
        } 

    }

    public void decreasePosition() {
        if (launchState == LauncherState.SPEAKER) {
        LauncherState.SPEAKER.position = LauncherState.SPEAKER.position - increment;
        } 

    }

    public boolean[] launcherConnections() {

        if (shootMotor1.getBusVoltage() != 0) {
            connections[0] = true;
        } else {
            connections[0] = false;
        }

        if (shootMotor1.getOutputCurrent() != 0) {
            connections[1] = true;
        } else {
            connections[1] = false;
        }

        if (shootMotor2.getBusVoltage() != 0) {
            connections[2] = true;
        } else {
            connections[2] = false;
        }

        if (shootMotor2.getOutputCurrent() != 0) {
            connections[3] = true;
        } else {
            connections[3] = false;
        }

        if (pivotMotor.getBusVoltage() != 0) {
            connections[4] = true;
        } else {
            connections[4] = false;
        }

        if (pivotMotor.getOutputCurrent() != 0) {
            connections[5] = true;
        } else {
            connections[5] = false;
        }

        if (flicker.getBusVoltage() != 0) {
            connections[6] = true;
        } else {
            connections[6] = false;
        }

        if (flicker.getOutputCurrent() != 0) {
            connections[7] = true;
        } else {
            connections[7] = false;
        }

        return connections;
    }

    public boolean hasBrownedOut() {
        return pivotMotor.getFault(FaultID.kBrownout);
    }

    public void printConnections() {
        SmartDashboard.putBoolean("shootMotor1 Voltage", connections[0]);
        SmartDashboard.putBoolean("shootMotor1 Current", connections[1]);

        SmartDashboard.putBoolean("shootMotor2 Voltage", connections[2]);
        SmartDashboard.putBoolean("shootMotor2 Current", connections[3]);

        SmartDashboard.putBoolean("Pivot Voltage", connections[4]);
        SmartDashboard.putBoolean("Pivot Current", connections[5]);

        SmartDashboard.putBoolean("Flicker Voltage", connections[6]);
        SmartDashboard.putBoolean("Flicker Current", connections[7]);
    }

    public static Launcher getInstance() {
        if (instance == null)
            instance = new Launcher();
        return instance;
    }
}