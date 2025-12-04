package frc.robot.Commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.CameraSystems.PhotonVisionCamera;
import java.time.Clock;

public class NetAlignCommand extends Command {

  private SwerveSubsystem swerve;
  private PhotonVisionCamera camera;

  private Clock timer = Clock.systemDefaultZone();
  private long startTime;

  private double tagYaw;
  private double tagPitch;

  private double speed = 0.005;

  public NetAlignCommand(SwerveSubsystem swerve, PhotonVisionCamera camera) {
    this.camera = camera;
    this.swerve = swerve;
    addRequirements(swerve, camera);
    startTime = timer.millis();
  }

  @Override
  public void execute() {
    getTagData();
    System.out.println("Running");
    swerve.drive(new Translation2d(getXtranslation(), getYtranslation()), getRotation(), false);
  }

  private double getXtranslation() {
    if (tagPitch > -4.5) return speed;
    if (tagPitch < -5.5) return -speed;
    return 0;
  }

  private double getYtranslation() {
    if (tagYaw > 1.5) return speed;
    if (tagYaw < -1.5) return -speed;
    return 0;
  }

  private double getRotation() {
    if (swerve.getHeading().getDegrees() > 1) return -speed;
    if (swerve.getHeading().getDegrees() < -1) return speed;
    return 0;
  }

  private void getTagData() {
    tagYaw = camera.getTargetYaw();
    tagPitch = camera.getTargetPitch();
  }

  @Override
  public boolean isFinished() {
    getTagData();
    if (!camera.hasTarget()) {
      System.out.println("No Target");
      return true;
    }
    if (timer.millis() - startTime >= 5000) {
      System.out.println("Timeout");
      return true;
    }
    return (getXtranslation() == 0 && getYtranslation() == 0 && getRotation() == 0);
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Done Aligning");
    // swerve.drive(new Translation2d(0, 0), getRotation(), false);
  }
}
