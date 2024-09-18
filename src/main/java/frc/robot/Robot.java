package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.AltRevLauncher;
import frc.robot.commands.AmpCommand;
import frc.robot.commands.AutoLeftShot;
import frc.robot.commands.AutoMidShot;
import frc.robot.commands.AutoPreload;
import frc.robot.commands.AutoRightShot;
import frc.robot.commands.AutoSpeaker;
import frc.robot.commands.BreakBeamHandoff;
import frc.robot.commands.Celebrate;
import frc.robot.commands.FoldOutCommand;

import frc.robot.commands.RevLauncher;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.IO.LED;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.Launcher.LauncherState;
import frc.robot.subsystems.launcher.Launcher.AmpMotorPos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.swerve.Drivebase;
import frc.robot.subsystems.swerve.Drivebase.DriveState;
import frc.robot.subsystems.vision.CameraSystem;
import org.littletonrobotics.junction.LoggedRobot;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

public class Robot extends LoggedRobot {
  //all the initialing for the systems
  private Drivebase drivebase;
  private Climber climber;
  private Intake intake;
  private Launcher launcher;
  private LED litty;
  private CameraSystem camSystem;
  
  private static XboxController driver;
  private static XboxController operator;
  //initialization of the auton chooser in the dashboard
  private Command m_autoSelected;

  private FoldOutCommand foldOutCommand;
  private BreakBeamHandoff handoffCommand;
  private ShootCommand shootCommand;
  private AutoSpeaker autoSpeaker;
  
  private AmpCommand ampCommand;

  

  //that is a chooser for the autons utilizing the sendableChooser which allows us to choose the auton commands
  private SendableChooser<Command> m_chooser = new SendableChooser<>();

  @Override
  public void robotInit() {
    drivebase = Drivebase.getInstance();
    launcher = Launcher.getInstance();
    intake = Intake.getInstance();
    climber = Climber.getInstance();
    litty = LED.getInstance();
    camSystem = CameraSystem.getInstance();
    // BackCam is at array position 0, FrontCam is at array positon 1
    camSystem.AddCamera(new PhotonCamera("BackCam"), new Transform3d(
        new Translation3d(-.31, .01, -0.375), new Rotation3d(0.0, Math.toRadians(30), Math.PI))
        );
    // camSystem.AddCamera(new PhotonCamera("FrontCam"),new Transform3d(
    //     new Translation3d(.325, -.275, 0.24), new Rotation3d(0.0, Math.toRadians(30), Math.toRadians(0.0))) 
    //     );


    driver = new XboxController(0);
    operator = new XboxController(1);
    //initializing the commands to use in this class
    handoffCommand = new BreakBeamHandoff();
    shootCommand = new ShootCommand();
    autoSpeaker = new AutoSpeaker();
    ampCommand = new AmpCommand();
    foldOutCommand = new FoldOutCommand();
   
    
    //all the commands to use in our autos. Not the paths to follow, but the shooting.
    NamedCommands.registerCommand("AutoSpeaker", autoSpeaker);
    NamedCommands.registerCommand("Handoff", new BreakBeamHandoff());
    NamedCommands.registerCommand("AutoLeftShot", new AutoLeftShot());
    NamedCommands.registerCommand("AutoRightShot", new AutoRightShot());
    NamedCommands.registerCommand("AutoMidShot", new AutoMidShot());
    NamedCommands.registerCommand("Celebrate", new Celebrate());
    NamedCommands.registerCommand("RevLauncher", new RevLauncher());
    NamedCommands.registerCommand("AutoPreload", new AutoPreload());
    NamedCommands.registerCommand("AltRevLauncher", new AltRevLauncher());

    //adding actual pathings to the chooser so we can select what to run during the match

    /*NAMING CONVENTION FOR THE PATHS: Px #y
     * WORKS FOR BOTH SIDES
     * x:
     * P1 - starts at the speaker at the side of the AMP station
     * P2 - middle of the speaker subwoofer
     * P3 - on the side of the source 
     * 
     * #- the amount of total rings the path shoots 
     * 3 - preload + 2 rings from the direction of the movement
     * 4 - preload + all of the rings that are not on the midline
     * 
     * y - direction of first pickup
     * L - goes to amp ring  (works on both sides)
     * R - goes to stage ring pickup  (Works on both sides)
     * M - goes to the middle one (Works on both sides)
     * if mix of 2 letters then it goes to letter 1 first then letter 2
     */
    m_chooser.addOption("P1 4L", new PathPlannerAuto("P1 4L"));
    m_chooser.addOption("P2 3L", new PathPlannerAuto("P2 3L"));
    m_chooser.addOption("P2 3ML", new PathPlannerAuto("P2 3ML"));
    m_chooser.addOption("P2 3MR", new PathPlannerAuto("P2 3MR"));
    m_chooser.addOption("P2 3R", new PathPlannerAuto("P2 3R"));
    m_chooser.addOption("P2 4L Mid", new PathPlannerAuto("P2 4L Mid"));
    m_chooser.addOption("P2 4L", new PathPlannerAuto("P2 4L"));
    m_chooser.addOption("P2 4R Mid", new PathPlannerAuto("P2 4R Mid"));
    m_chooser.addOption("P2 4R", new PathPlannerAuto("P2 4R"));
    m_chooser.addOption("VtestAuto", new PathPlannerAuto("VtestAuto"));
    m_chooser.addOption("Help", new PathPlannerAuto("Help"));
    m_chooser.addOption("Mid:ine", new PathPlannerAuto("MidLine"));
    m_chooser.addOption("FreakyMid", new PathPlannerAuto("FreakyMid"));
    m_chooser.addOption("P2 Middle", new PathPlannerAuto("P2 Middle"));
    m_chooser.addOption("P3 4R", new PathPlannerAuto("P3 4R"));
    m_chooser.addOption("P3UnderStage", new PathPlannerAuto("P3UnderStage"));
    SmartDashboard.putData("Auto choices", m_chooser);

  }

  @Override

  public void robotPeriodic() {
        Pose2d cameraPositionTele = camSystem.calculateRobotPosition();

       Pose2d posTele = drivebase.updateOdometry(cameraPositionTele);

        SmartDashboard.putNumber("Odometry X", posTele.getX());
        SmartDashboard.putNumber("Odometry Y", posTele.getY());

      //this is getting the data from the cameras through the cameraSystem class 
     if (camSystem.getCamera(0).isConnected()) {
            PhotonPipelineResult backResult = camSystem.getResult(0);      
            if (backResult.hasTargets()) {
                PhotonTrackedTarget target = backResult.getBestTarget();
                Transform3d bestCameraToTarget = target.getBestCameraToTarget();
                double distance  = bestCameraToTarget.getTranslation().getNorm();
                SmartDashboard.putString("Back Camera Target", "Yes Targets");
                SmartDashboard.putNumber("Back to Target", distance);
                SmartDashboard.putNumber("Back Camera Target Yaw", target.getYaw());
                SmartDashboard.putNumber("Back Camera Target Pitch", target.getPitch());
                SmartDashboard.putNumber("Back Camera Target Area", target.getArea());
                SmartDashboard.putNumber("ID", target.getFiducialId());

            } else if(backResult.hasTargets() == false) {
                SmartDashboard.putString("Back Camera Target", "No Targets");
            }
        } 
        



      
      //testing the valuies that the camera gives us and outputing it into the dashboard
      Pose2d cameraPosition = camSystem.calculateRobotPosition();
      SmartDashboard.putNumber("Camera X Position", cameraPosition.getX());
      SmartDashboard.putNumber("Camera Y Position", cameraPosition.getY());
      SmartDashboard.putNumber("Camera Heading", cameraPosition.getRotation().getDegrees());
    
    CommandScheduler.getInstance().run();
    drivebase.periodic();

    //putting all of the info from the subsystems into the dashvoard so we can test things
    SmartDashboard.putNumber("Gyro Angle:", (drivebase.getHeading() + 90) % 360);
    SmartDashboard.putNumber("X-coordinate", drivebase.getPose().getX());
    SmartDashboard.putNumber("Y-coordinate", drivebase.getPose().getY());

    SmartDashboard.putNumber("Flipper Position", intake.getFlipperPosition());
    SmartDashboard.putNumber("Launcher Position", launcher.getPosition());

    SmartDashboard.putString("Intake State", intake.getIntakeState().toString());
    SmartDashboard.putString("Launcher State", launcher.getLaunchState().toString());

    SmartDashboard.putBoolean("Launcher Breakbeam", launcher.getBreakBeam());
    SmartDashboard.putBoolean("Intake Breakbeam", intake.getBreakBeam());

    SmartDashboard.putBoolean("Brownout", launcher.hasBrownedOut());
    SmartDashboard.putNumber("Speaker Position", launcher.getSpeakerPosition());



    SmartDashboard.putNumber("LeBron Position", launcher.getAmpPostion());
  }

  @Override
  public void autonomousInit() {
    //getting the value we chose from the dashboard and putting it into motion in the auton
    m_autoSelected = m_chooser.getSelected();

    drivebase.resetOdometry(PathPlannerAuto.getStaringPoseFromAutoFile(m_chooser.getSelected().getName()));
    
    
//schedules the command so it actually begins moving
    if (m_autoSelected != null) {
      m_autoSelected.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    //updating the intake for the autointake command
    intake.updatePose();
    //using cameras to calculate the robot position instead of odometry.
    //we use a mix of odometry + camera positions to calculate the robot position
    Pose2d cameraPosition = camSystem.calculateRobotPosition(); 
    Pose2d pose = drivebase.updateOdometry(cameraPosition);

    SmartDashboard.putNumber("Auto X", drivebase.getPose().getX());
    SmartDashboard.putNumber("Auto Y", drivebase.getPose().getY());
    SmartDashboard.putNumber("Odometry X", pose.getX());
    SmartDashboard.putNumber("Odometry Y", pose.getY());
  }

  @Override
  public void teleopInit() {
    //as soon as we begin teleop we desable the auton selection
    litty.setBlue();
    if (m_autoSelected != null) {
      m_autoSelected.cancel();
    }

    Translation2d testxy = new Translation2d(16.57 - 14.7, 5.54);
    Rotation2d testRot = new Rotation2d(0);
    Pose2d test = new Pose2d(testxy, testRot);
    drivebase.resetOdometry(test);
  }

  @Override
  public void teleopPeriodic() {
    intake.updatePose();

    /* DRIVE CONTROLS */

    if (!ampCommand.isScheduled()) {
      launcher.moveAmp();
    }
    //setting inputs for driving through the driver controller
    double ySpeed = drivebase.inputDeadband(-driver.getLeftX());
    double xSpeed = drivebase.inputDeadband(driver.getLeftY());
    double rot = drivebase.inputDeadband(-driver.getRightX());
    //using buttons to rotate the robot by increments of 90 degrees
    // if (driver.getAButton()) {
    //   drivebase.currHeading = -1;
    //   drivebase.rotateTo(xSpeed, ySpeed, 180);
    // } else if (driver.getBButton()) {
    //   drivebase.currHeading = -1;
    //   drivebase.rotateTo(xSpeed, ySpeed, 270);
    // } else if (driver.getYButton()) {
    //   drivebase.currHeading = -1;
    //   drivebase.rotateTo(xSpeed, ySpeed, 0);
    // } else if (driver.getXButton()) {
    //   drivebase.currHeading = -1;
    //   drivebase.rotateTo(xSpeed, ySpeed, 90);
    // } else if (driver.getLeftTriggerAxis() > 0) {
    // } else {
    //   drivebase.currHeading = -1;
    //   drivebase.drive(xSpeed, ySpeed, rot);
    // }
    

    

    if (driver.getPOV() == 0) {
      drivebase.zeroHeading();
    }
    if(driver.getPOV() == 180){
      litty.setRed();
    }
    if (operator.getYButton()) {
      launcher.setLauncherState(LauncherState.SPEAKER);
      launcher.updatePose();
      launcher.setReverseLauncherOn();
      launcher.setFlickerReverse();
    }

    if (driver.getRightTriggerAxis() > 0) {
      drivebase.setDriveState(DriveState.SLOW);
    } else if (!CommandScheduler.getInstance().isScheduled(ampCommand)) {
      drivebase.setDriveState(DriveState.NORMAL);
    }
    //getting yaw from the tag to rotate towards it. The robot will allign itself with the 
    if(driver.getLeftTriggerAxis() > 0)
    {
      Double yaw = camSystem.getYawForTag(0);
        if(yaw !=null)
        {
          rot =  -yaw * .002 * Constants.DriveConstants.kMaxAngularSpeed;
        }
    }
    drivebase.drive(xSpeed, ySpeed, rot);
    /* OPERATOR CONTROLS */
    /*Operator controller map
     * Y - launching the ring out of the shooter
     * B - Used for testing in the pit. Just spits out the ring.
     * A - Folds out the robot if the ring is stuck or something
     * X - Intake from the launcher (source type intake)
     *
     * RB - starts the handoff command
     * LB - brings the robot back to hover mode
     *
     * LeftStick - decrease the launcher position
     * RightStick - increase the launcher position
     *
     * UP Arrow - setting launcher to speaker position
     * Right Arrow - setting launcher to amp position
     * Down Arrow - setting the launcher to tossing position
     * Left Arrow - Longshot position
     *
     * Left back button - cancels every command on the robot
     * right back button - sets the robot up to amp
     */
    if (operator.getRightBumper()) {
      handoffCommand.schedule();
    }

    if (operator.getBButton()) {
      launcher.setLauncherState(LauncherState.TEST);
      launcher.eject();
      launcher.setFlickerPartial();
      litty.setRed();
    }

    if (operator.getLeftBumper()) {
      intake.setIntakeState(IntakeState.STOP); 
      launcher.setLauncherState(LauncherState.HOVER);
      launcher.setAmpPose(AmpMotorPos.DOWN);
      launcher.updatePose();
      launcher.moveAmp();
      launcher.setLauncherOff();
      launcher.setFlickOff();
    }

    if (operator.getRightStickButtonPressed()) {
      launcher.increasePosition();
    }
    if(operator.getLeftStickButtonPressed()){
      launcher.decreasePosition();
    }

    // *CLIMBER CONTROLS */

    if (driver.getRightBumper()) {
      climber.setClimbingPower();
    } else if (driver.getLeftBumper()) {
      climber.setReverseClimberPower();
    } else {
      climber.setClimberOff();
    }

    /* LAUNCHER CONTROLS */
    if (operator.getPOV() == 0) {
      launcher.setLauncherState(LauncherState.SPEAKER);
    }
    if (operator.getPOV() == 90) {
      launcher.setLauncherState(LauncherState.AMP);
    }
    if (operator.getPOV() == 180) {
      launcher.setLauncherState(LauncherState.TOSS);
    }
    if (operator.getPOV() == 270) {
      launcher.setLauncherState(LauncherState.LONG);
    }
     if (operator.getAButton()) {
      foldOutCommand.schedule();
      intake.setIntakeState(IntakeState.GROUND);
     }
    if (operator.getXButton()) {
      launcher.setLauncherState(LauncherState.TEST);
      launcher.setReverseLauncherOn();
      intake.setReverseRollerPower();
      launcher.setFlickerReverse();
      
    }

    if (operator.getRightTriggerAxis() > 0) {
      if (launcher.getLaunchState() == LauncherState.AMP) {
        ampCommand.initialize();
        ampCommand.schedule();
        drivebase.setDriveState(DriveState.SLOW);
      } else if (launcher.getLaunchState() == LauncherState.ALTAMP) {
        
        drivebase.setDriveState(DriveState.SLOW);
      } else {
        shootCommand.initialize();
        shootCommand.schedule();
      }
    } else if (operator.getLeftTriggerAxis() > 0) {
      foldOutCommand.cancel();
      launcher.setLauncherOff();
      launcher.setFlickOff();
      intake.setRollerOff();
      shootCommand.cancel();
      ampCommand.cancel();
      handoffCommand.cancel();
    }
    
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }

}