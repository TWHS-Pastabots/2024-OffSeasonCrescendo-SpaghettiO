package frc.robot.subsystems.launcher;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.LauncherConstants;
import frc.robot.subsystems.IO.DigitalInputs;
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

    public Launcher() {

        //Initialization of all the motors - setting power and other settings

        //Some motors are inverted some others, double check with trial and error 
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
        
        pivotMotor.setOpenLoopRampRate(1);

        ampMotor = new CANSparkMax(Ports.lebron, MotorType.kBrushless);
        ampMotor.restoreFactoryDefaults();

        ampMotor.setSmartCurrentLimit(20);
        ampMotor.setIdleMode(IdleMode.kBrake);

        //PID calculations. 
        //PIVOT MOTOR PID CALCULATIONS
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

        //AMP PID SETUP
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
        //the breakbeam instance from an input
        breakBeam = DigitalInputs.getInstance();

        

    }
    //launcher update method. Updates the launcher's positiong to the specified position
    public void updatePose() {

        pivotController1.setReference(launchState.position, CANSparkMax.ControlType.kPosition, 0,
                feedForward.calculate(encoder.getPosition(), 0));

        

    }
    //moves the amp mechanism to the specified position
    public void moveAmp() {
        ampMotorController.setReference(ampPose.position, CANSparkMax.ControlType.kPosition, 0,
                ampMotorFeedForward.calculate(boxScore.getPosition(), 0));
    }
    //motor speeds for ejecting the ring out of the launcher. Only one should be on to reduce the speed so the eject is safe.
    public void eject() {
        shootMotor2.set(0);
        shootMotor1.set(launchState.launchSpeed);
    }
    //The power needed for the amp motor
    public void ampOn() {
        ampMotor.set(-0.5);
    }
    //The power needed to move in reverese
    public void ampReverse() {
        ampMotor.set(0.5);
    }
    //Amp rest state
    public void ampOff() {
        ampMotor.set(0.0);
    }
    //launcher rotation at rest
    public void setPivotOff() {
        pivotMotor.set(0.0);
    }
    //getting the test position for the launcher (can be changed on dashboard during testing)
    public double getTestPosition() {
        return LauncherState.TEST.position;
    }
    //getting the speaker positiong (operator can change mid match therefore need this)
     public double getSpeakerPosition() {
        return LauncherState.SPEAKER.position;
    }
    //the amp position needed to score the amp 
    public double getAmpPostion() {
        return boxScore.getPosition();
    }
    //the electrical component of the amp, mostly to output to the dashboard to understand whether or not amp is working
    public double getAmpCurrent(){
        return ampMotor.getOutputCurrent();

    }

    

    
    //Giving the launcher power, everything gets 100% but the amp that only needs 10% to work
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
    //The reverse of Launcher power
    public void setReverseLauncherOn() {

        
        shootMotor1.set(-launchState.launchSpeed);
        shootMotor2.set(-launchState.launchSpeed);
        
    }
    //Launcher turned off
    public void setLauncherOff() {
        shootMotor1.set(0.0);
        shootMotor2.set(0.0);
    }
    //Flickers are the small wheels at the back of the launcher. They are used to push the rings to the flywheels
    //and intaking the rings that are 
    public void setFlickerOn() {
        flicker.set(1.0);
    }
    //setting flickers in reverse
    public void setFlickerReverse() {
        flicker.set(-1.0);
    }
    //set it to partial speed, mostly for testing in the pits and such
    public void setFlickerPartial() {
        flicker.set(0.85);
    }

    public void setFlickOff() {
        flicker.set(0);
    }
    //get the position of the robot according to the encoders. Not reliable, needs vision
    public double getPosition() {
        return encoder.getPosition();
    }
    //setting the opposite of the breakbeam because normally if the breakbeam is broken it would give a false,
    //so now it gives true if broken, false if not broken
    public boolean getBreakBeam() {
        return !breakBeam.getInputs()[Ports.launcherBreakBeam];
    }
   
    //get the current that the pivot motor is receiving (used for testing)
    public double getPivotCurrent() {
        return pivotMotor.getOutputCurrent();
    }
    //returns if the position of any of the launcher subsystems has reached a point that makes sense
    public boolean hasReachedPose(double tolerance) {
        return Math.abs(getPosition() - launchState.position) < tolerance;
    }
     //getting the current launcherstate 
    public LauncherState getLaunchState() {
        return launchState;
    }
    //setting the launcherstate from the controller
    public void setLauncherState(LauncherState state) {
        launchState = state;
    }
    //setting the Amp position to the position that we passed along to it
    public void setAmpPose(AmpMotorPos position) {
        ampPose = position;
    }
    //increasing and decreasing the increments that the operator can change the positions of the speaker by
    public void increaseIncrement() {
        increment += 0.25;
    }

    public void decreaseInrement() {
        increment -= 0.25;
    }
    //increases the speaker position in the middle of the match
    public void increasePosition() {
        
        if (launchState == LauncherState.SPEAKER) {
            LauncherState.SPEAKER.position = LauncherState.SPEAKER.position + increment;
        } 

    }
    //same thing as above but decreasing
    public void decreasePosition() {
        if (launchState == LauncherState.SPEAKER) {
        LauncherState.SPEAKER.position = LauncherState.SPEAKER.position - increment;
        } 

    }
    //this method checks if the launcher is connected to the power supply. It runs through all the motors that are used in the 
    //launcher class. Returns true if everything is receiving power and false if one thing is broken
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
        //returns the boolean, only one
        return connections;
    }
    //checking for a brownout
    public boolean hasBrownedOut() {
        return pivotMotor.getFault(FaultID.kBrownout);
    }
    //outputting the power data onto the smartdashboard
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
    //simple instance method to use in every other class
    public static Launcher getInstance() {
        if (instance == null)
            instance = new Launcher();
        return instance;
    }
}