package frc.robot.Commands;

import java.util.function.Supplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Subsystems.SwerveSubsystem;

public class FineAlignToTag extends Command {

    private final SwerveSubsystem drive;
    private final Supplier<Pose2d> targetPoseSupplier;

    // Tunable gains
    private final PIDController forwardPID = new PIDController(1.8, 0.0, 0.1);
    private final PIDController strafePID  = new PIDController(1.8, 0.0, 0.1);
    private final PIDController thetaPID   = new PIDController(4.5, 0.0, 0.15);

    public FineAlignToTag(SwerveSubsystem drive, Supplier<Pose2d> targetPoseSupplier) {
        this.drive = drive;
        this.targetPoseSupplier = targetPoseSupplier;
        addRequirements(drive);

        thetaPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        forwardPID.reset();
        strafePID.reset();
        thetaPID.reset();
    }

    @Override
    public void execute() {

        Pose2d robotPose = drive.getPose();
        Pose2d targetPose = targetPoseSupplier.get();

        // Compute pose error in robot-relative coordinates
        Pose2d error = targetPose.relativeTo(robotPose);

        double xError = error.getX();                   // forward error (+ means target is in front)
        double yError = error.getY();                   // left error (+ means target is left)
        double thetaError = error.getRotation().getRadians();

        // Calculate robot-relative speeds
        double vx = forwardPID.calculate(xError, 0.0);
        double vy = strafePID.calculate(yError, 0.0);
        double omega = thetaPID.calculate(thetaError, 0.0);

        // Clamp speeds to safe precision alignment values
        vx = Math.min(1.0, Math.max(-1.0, vx));
        vy = Math.min(1.0, Math.max(-1.0, vy));
        omega = Math.min(2.0, Math.max(-2.0, omega));

        drive.drive(new ChassisSpeeds(vx, vy, omega));
    }

    @Override
    public boolean isFinished() {
        return false; // run until interrupted
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(new ChassisSpeeds(0,0,0));
    }
}