package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.DrivebaseCommands.DriveToPose;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.ElasticSubsystem;
import frc.robot.Subsystems.LightsSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Utilites.Constants;
import frc.robot.Utilites.Elastic;
import frc.robot.Utilites.FieldLayout;
import frc.robot.Utilites.HelperFunctions;
import frc.robot.Utilites.LEDRequest;
import frc.robot.Utilites.LEDRequest.LEDState;
import frc.robot.Utilites.LimelightHelpers;
import frc.robot.Utilites.Constants.DrivebaseConstants;
import frc.robot.Utilites.Constants.OperatorConstants;
import frc.robot.Utilites.Constants.PWMPorts;
import java.io.File;
import java.lang.reflect.Field;
import java.util.function.DoubleSupplier;
import com.pathplanner.lib.auto.NamedCommands;


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

    CommandXboxController driverXbox = new CommandXboxController(Constants.XBOX_PORT);
    LightsSubsystem lights = new LightsSubsystem(PWMPorts.LIGHT_PORT, Constants.LIGHTS_AMOUNT);
    ElasticSubsystem elasticSubsystem = new ElasticSubsystem();
    PowerDistribution PDH = new PowerDistribution(20, ModuleType.kRev);
    FieldLayout field = new FieldLayout();
    ArmSubsystem arm;

    Pose2d testPose = new Pose2d(0, 0, new Rotation2d(0));

    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve/neo"));

    boolean doRejectUpdate;

    Timer t = new Timer();
    double crazySpinSpeedModifier = 1;
    boolean isCrazySpinning = false;

    Pose2d targetPose = new Pose2d(2, 5, new Rotation2d(0));
    private PIDController forwardPID = new PIDController(3, 0, 0.001);
    private PIDController strafePID = new PIDController(3, 0, 0.001);
    private PIDController thetaPID = new PIDController(0.05, 0, 0.001);

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

    SwerveInputStream driveDirectAngleCrazySpin = driveAngularVelocity.copy()
            .withControllerHeadingAxis(getFakeX(),
                    getFakeY())
            .headingWhile(true);

    public RobotContainer() {

        arm = new ArmSubsystem();


        targetPose = field.getPoseInFrontOfTag(18, 1.5);
        elasticSubsystem.putAutoChooser();
        registerNamedCommands();
        configureBindings();

    }

    private void configureBindings() {
        // Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
        // Command driveFieldOrientedCrazySpin = drivebase.driveFieldOriented(driveDirectAngleCrazySpin);
        // drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);

        // driverXbox.rightTrigger(0.2).whileTrue(new StartEndCommand(() -> {
        //     drivebase.setCreepDrive(true);
        // }, () -> {
        //     drivebase.setCreepDrive(false);
        // }));

        // driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));

        // // Crazy spin of doom
        // driverXbox.leftTrigger(0.7).whileTrue(driveFieldOrientedCrazySpin
        //         .alongWith(new StartEndCommand(() -> isCrazySpinning = true, () -> isCrazySpinning = false))
        //         .alongWith(Commands.runOnce(() -> {
        //             drivebase.setMaximumSpeeds(DrivebaseConstants.CRAZY_SPIN_SPEED,
        //                     DrivebaseConstants.MAX_ANGULAR_VELOCITY);
        //         })))
        //         .onTrue(new InstantCommand(() -> {
        //             t.reset();
        //             t.start();
        //         })).onFalse(new InstantCommand(() -> drivebase.setMaximumSpeeds(DrivebaseConstants.MAX_SPEED,
        //                 DrivebaseConstants.MAX_ANGULAR_VELOCITY)));

        
        // driverXbox.b().onTrue(new DriveToPose(drivebase, () -> targetPose, forwardPID, strafePID, thetaPID,
        //         this::driverOverride, lights));

        driverXbox.y().onTrue(new InstantCommand(() -> arm.togglePosition()));


    }

    private boolean driverOverride() {
        double x = driverXbox.getRightX();
        double y = driverXbox.getRightY();
        double threshold = 0.5;

        return Math.abs(x) > threshold || Math.abs(y) > threshold;
    }

    public void enabledInit() {
        t.reset();
        t.start();
    }

    public void enabledPerodic() {

        // if (isCrazySpinning) {
        //     if (driverXbox.getRightY() > OperatorConstants.DEADBAND) {
        //         crazySpinSpeedModifier -= 0.05; // Inverted because the controller axis is inverted
        //         t.reset();
        //         t.start();
        //     } else if (driverXbox.getRightY() < -OperatorConstants.DEADBAND) {
        //         t.reset();
        //         t.start();
        //         crazySpinSpeedModifier += 0.05;
        //     }
        // }
        // crazySpinSpeedModifier = HelperFunctions.clamp(crazySpinSpeedModifier, -7.5,
        //         7.5);

        arm.zeroEncoderPeriodic();
        arm.run();
        // arm.runWithController(-driverXbox.getLeftY());

    }

    private DoubleSupplier getFakeX() {
        return () -> Math.cos(crazySpinSpeedModifier * t.get());
    }

    private DoubleSupplier getFakeY() {
        return () -> Math.sin(crazySpinSpeedModifier * t.get());
    }

    public void robotPerodic() {
        sendDashboardData();
        setLights();
        lights.run();

        SmartDashboard.putData("Forward PID", forwardPID);
        SmartDashboard.putData("Strafe PID", strafePID);
        SmartDashboard.putData("Theta PID", thetaPID);

    }

    public void sendDashboardData() {
        ElasticSubsystem.putBoolean("Rejecting Telemetry Updates", doRejectUpdate);
        ElasticSubsystem.putColor("Lights", HelperFunctions.convertToGRB(lights.getLEDRequest().getColour()));
        ElasticSubsystem.putString("Target Pose", targetPose.toString());
        ElasticSubsystem.putString("Robot Pose", drivebase.getPose().toString());
        ElasticSubsystem.putNumber("Total Current Pull", PDH.getTotalCurrent());
        ElasticSubsystem.putNumber("Crazy Spin Modifier", crazySpinSpeedModifier);
        ElasticSubsystem.putBoolean("Is Creep Drive", drivebase.getCreepDrive());

    }

    public void setupDashboard() {
        ElasticSubsystem.putBoolean("Lights Switch", true);
    }

    public void setLights() {
        if (drivebase.getCreepDrive())
            lights.requestLEDState(new LEDRequest(LEDState.SOLID).withColour(HelperFunctions.convertToGRB(Color.kRed))
                    .withPriority(4).withBlinkRate(0.7));
        else
            lights.requestLEDState(new LEDRequest(LEDState.SOLID).withColour(HelperFunctions.convertToGRB(Color.kGreen))
                    .withPriority(5));
        if (isCrazySpinning)
            lights.requestLEDState(new LEDRequest(LEDState.BLINK)
                    .withColour(HelperFunctions.convertToGRB(Color.kPurple)).withPriority(0).withBlinkRate(0.2));

        if (!ElasticSubsystem.getBoolean("Lights Switch")) {
            lights.requestLEDState(new LEDRequest(LEDState.OFF).withPriority(-999));
        }

        if (DriverStation.isDisabled())
            lights.requestLEDState(new LEDRequest(LEDState.RAINBOW).withPriority(-1));
    }

    public void updateTelemetry() {
        try {
            doRejectUpdate = false;

            LimelightHelpers.SetRobotOrientation(
                    "limelight",
                    drivebase.getHeading().getDegrees(),
                    0, 0, 0, 0, 0);

            LimelightHelpers.PoseEstimate robotPosition = LimelightHelpers
                    .getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

            // reject vision while spinning too fast
            if (Math.abs(drivebase.getRobotVelocity().omegaRadiansPerSecond) > Math.toRadians(120)) {
                doRejectUpdate = true;
            }

            if (robotPosition.tagCount < 1) {
                doRejectUpdate = true;
            }

            if (!doRejectUpdate) {
                drivebase.setVisionStdDevs(VecBuilder.fill(1.5, 1.5, 99999999)); // Reject vision rotation
                drivebase.updateBotPose(robotPosition.pose);
            }

        } catch (Exception e) {

            System.out.println("NO DATA FROM LIMELIGHT | " + e.getLocalizedMessage());
        }
    }

    public Command getAutonomousCommand() {
        return drivebase.getAutonomousCommand(elasticSubsystem.getSelectedAuto());
    }

    public void setDashboardTab(String name) {
        Elastic.selectTab(name);
    }

    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake);
    }

    public void registerNamedCommands() {
        NamedCommands.registerCommand("Drive to Test Pose", new DriveToPose(drivebase, () -> targetPose, forwardPID, strafePID, thetaPID, () -> false, lights));
    }

}