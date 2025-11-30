package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
//https://frc-elastic.gitbook.io/docs/additional-features-and-references/remote-layout-downloading
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.LightsSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Utilites.Elastic;
import frc.robot.Utilites.LEDRequest;
import frc.robot.Utilites.LEDRequest.LEDState;
import frc.robot.Utilites.LimelightHelpers;
import frc.robot.Utilites.Constants.OperatorConstants;

import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

    private static RobotContainer instance;

    public static RobotContainer getInstance() {
        if (instance == null)
            instance = new RobotContainer();
        return instance;
    }

    final CommandXboxController driverXbox = new CommandXboxController(0);
    LimelightSubsystem limelight;
    LightsSubsystem lights = new LightsSubsystem(0, 50);
    SendableChooser<String> autoChooser = new SendableChooser<>();
    Pose2d testPose = new Pose2d(2.0, 4, new Rotation2d(0)); // like 1 m in front of tag 18 (Reefscape)

    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve/neo"));

    boolean targetVisible = false;
    double botX;
    double botY;

    /**
     * // * Converts driver input into a field-relative ChassisSpeeds that is
     * controlled
     * // * by angular velocity.
     * //
     */
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
            () -> driverXbox.getLeftY() * -1,
            () -> driverXbox.getLeftX() * -1)
            .withControllerRotationAxis(driverXbox::getRightX)
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);

    // /**
    // * Clone's the angular velocity input stream and converts it to a
    // fieldRelative
    // * input stream.
    // */
    SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
            .withControllerHeadingAxis(() -> -driverXbox.getRightX(),
                    () -> -driverXbox.getRightY())
            .headingWhile(true);

    public RobotContainer() {
        autoChooser.setDefaultOption("Nothing", "Nothing");
        autoChooser.addOption("Move left", "Move left");
        autoChooser.addOption("Move right", "Move right");
        SmartDashboard.putData("Select Auto", autoChooser);

        limelight = new LimelightSubsystem("limelight");
        configureBindings();
    }

    private void configureBindings() {
        // TODO Make creep drive not work in autoaligning? and check obstacle avoidance
        Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
        drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);

        // Zero drivebase gyro
        driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));

        // Drive to selected pose
        driverXbox.b().onTrue(new ParallelCommandGroup(drivebase.driveToPose(testPose),
                new InstantCommand(() -> lights.requestLEDState(
                        new LEDRequest(LEDState.BLINK).withBlinkRate(1000).withColour(Color.kWhite).withPriority(2)))));

        // While in creep drive
        driverXbox.rightTrigger(0.2).whileTrue(Commands.runOnce(() -> {
            drivebase.setCreepDrive(true);
            lights.requestLEDState(new LEDRequest(LEDState.SOLID).withColour(Color.kYellow).withPriority(4));
        }).repeatedly());

        // While not in creep drive
        driverXbox.rightTrigger(0.2).whileFalse(Commands.runOnce(() -> {
            drivebase.setCreepDrive(false);
            lights.requestLEDState(new LEDRequest(LEDState.SOLID).withColour(Color.kGreen).withPriority(5));
        }).repeatedly());

    }

    public void enabledPerodic() {
    }

    public void robotPerodic() {
        lights.run();
    }

    public void updateTelemetry() {
        LimelightHelpers.SetRobotOrientation("limelight", drivebase.getHeading().getDegrees(), 0, 0, 0, 0, 0);
        targetVisible = LimelightHelpers.getTV("limelight");
        if (targetVisible) {
            LimelightHelpers.PoseEstimate pose = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
            if (pose.rawFiducials.length == 0)
                return;
            drivebase.updateBotPose(pose.pose);
        }
    }

    public Command getAutonomousCommand() {
        return drivebase.getAutonomousCommand(autoChooser.getSelected());
    }

    public void setDashboardTab(String name) {
        Elastic.selectTab(name);
    }

    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake); // Dont use this for drivebase
        // Use this for mechanisms, For example: after 10 sec once the game finishes,
        // put the lifting arm to neutral
        // (So you can take it off something)
    }
}