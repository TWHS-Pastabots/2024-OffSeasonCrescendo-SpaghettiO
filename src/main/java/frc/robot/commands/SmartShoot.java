// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.Launcher.LauncherState;
import frc.robot.subsystems.vision.CameraSystem;

public class SmartShoot extends Command {
  private boolean ended;
  private Launcher launcher;
  private CameraSystem camSystem;

  private Double targetAngle;

  
  private double startTime;
  private double elapsedTime;
  private double windup = 0.2;
  private double duration = windup + .1;

  
  public SmartShoot() {
    launcher = Launcher.getInstance();
    camSystem = CameraSystem.getInstance();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ended = false;

    launcher.setLauncherState(LauncherState.AUTO);
    startTime = -1;
    elapsedTime = 0;
    launcher.setLauncherOn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    targetAngle = camSystem.getTargetAngle(1, 4);
    if(targetAngle != null)
    {
      launcher.setLauncherPosition(targetAngle);
    }
    launcher.updatePose();

    if(launcher.hasReachedPose(1.2)){
    launcher.setLauncherOn();
    launcher.setFlickerOn();

      if (startTime == -1) {
        startTime = Timer.getFPGATimestamp();
      }
      elapsedTime = Timer.getFPGATimestamp() - startTime;

      if (elapsedTime > duration) {
        ended = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    launcher.setFlickOff();
    launcher.setLauncherOff();
    launcher.setLauncherState(LauncherState.HOVER);
    launcher.updatePose();  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ended;
  }
}
