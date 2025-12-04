package frc.robot.Commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.SwerveSubsystem;

public class DriveHorizontalCommand extends Command {

  private SwerveSubsystem swerve;

  private double speed = 0.2;
  private double distance = 0;
  private double distanceTraveled = 0;

  public DriveHorizontalCommand(SwerveSubsystem swerve, double distance) {
    this.swerve = swerve;
    this.distance = distance;

    addRequirements(swerve);
  }

  @Override
  public void execute() {
    double xTranslation = getXtranslation();
    distanceTraveled += Math.abs(xTranslation) / 20;
    swerve.drive(new Translation2d(0, -xTranslation), 0, false);
  }

  private double getXtranslation() {
    return speed * Math.abs(distance) / distance;
  }

  @Override
  public boolean isFinished() {
    return (distanceTraveled >= Math.abs(distance));
  }

  @Override
  public void end(boolean interrupted) {}
}
