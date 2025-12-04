package frc.robot;

//https://frc-elastic.gitbook.io/docs/additional-features-and-references/remote-layout-downloading
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.ElasticSubsystem;
import frc.robot.Subsystems.LightsSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Utilites.Constants;
import frc.robot.Utilites.Elastic;
import frc.robot.Utilites.FieldLayout;
import frc.robot.Utilites.HelperFunctions;
import frc.robot.Utilites.LEDRequest;
import frc.robot.Utilites.LEDRequest.LEDState;
import frc.robot.Utilites.LimelightHelpers;
import frc.robot.Utilites.Constants.OperatorConstants;
import frc.robot.Utilites.Constants.PWMPorts;
import frc.robot.Utilites.Elastic.Notification;
import frc.robot.Utilites.Elastic.NotificationLevel;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;
import com.reduxrobotics.canand.ReduxJNI.Helper;

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

    final CommandXboxController driverXbox = new CommandXboxController(Constants.XBOX_PORT);
    LimelightSubsystem limelight;
    LightsSubsystem lights = new LightsSubsystem(PWMPorts.LIGHT_PORT, Constants.LIGHTS_AMOUNT);
    ElasticSubsystem elasticSubsystem = new ElasticSubsystem();
    FieldLayout fieldLayout = new FieldLayout();
    
    Pose2d testPose = new Pose2d(2.0, 4, new Rotation2d(0)); // roughly 1.5 m in front of tag 18 (Reefscape)

    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve/neo"));

    boolean targetVisible = false;
    double botX;
    double botY;
    boolean isCreepDrive = false;
    boolean isAligning = false;
    double testValue = 1;
    Pose2d targetPose = new Pose2d(2,4, new Rotation2d(0));

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


        elasticSubsystem.putAutoChooser();
        limelight = new LimelightSubsystem("limelight");
        registerNamedCommands();
        configureBindings();
        targetPose = fieldLayout.getOffsetedPosFromTag(18);
    }

    private void configureBindings() {
        // TODO Make creep drive not work in autoaligning? and check obstacle avoidance
        Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
        drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);

        // Zero drivebase gyro
        driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));

        // Drive to selected pose
        // driverXbox.b().onTrue(new ParallelCommandGroup(drivebase.driveToPose(targetPose),
        //         new InstantCommand(() -> isAligning = true)).andThen(() -> isAligning = false));


        // While in creep drive
        driverXbox.rightTrigger(0.2).whileTrue(Commands.runOnce(() -> {
            drivebase.setCreepDrive(true);
            isCreepDrive = true;
        }).repeatedly());

        // While not in creep drive
        driverXbox.rightTrigger(0.2).whileFalse(Commands.runOnce(() -> {
            drivebase.setCreepDrive(false);
            isCreepDrive = false;
        }).repeatedly());

        driverXbox.y().onTrue(new InstantCommand(() -> {
            setupDashboard();
            testValue += 1;
        }));

    }

    public void enabledPerodic() {

        if(isCreepDrive) lights.requestLEDState(new LEDRequest(LEDState.SOLID).withColour(HelperFunctions.convertToGRB(Color.kRed)).withPriority(4).withBlinkRate(0.7));
        else lights.requestLEDState(new LEDRequest(LEDState.SOLID).withColour(HelperFunctions.convertToGRB(Color.kGreen)).withPriority(5));
        if(isAligning) lights.requestLEDState(new LEDRequest(LEDState.SOLID).withColour(HelperFunctions.convertToGRB(Color.kWhiteSmoke)).withPriority(2).withBlinkRate(0.4));
    }

    public void robotPerodic() {
        sendDashboardData();
        lights.run();
        //sendNotificationsPerodic();
        if(!ElasticSubsystem.getBoolean("LightsSwitch")){
            lights.requestLEDState(new LEDRequest(LEDState.OFF).withPriority(-1));
        }

        if(ElasticSubsystem.getNumber("TestValue") != testValue){
            testValue = ElasticSubsystem.getNumber("TestValue");
        }

    }

    public void sendDashboardData(){
        ElasticSubsystem.putBoolean("Has April tag", targetVisible);
        ElasticSubsystem.putColor("Lights", HelperFunctions.convertToGRB(lights.getLEDRequest().getColour()));
        ElasticSubsystem.putString("Target Pose", targetPose.toString());
        ElasticSubsystem.putString("Robot Pose", drivebase.getPose().toString());
        
    }


    public void setupDashboard(){
        ElasticSubsystem.putBoolean("LightsSwitch", true);
        ElasticSubsystem.putNumber("TestValue", testValue);
    }

    public void setLights(){
        
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
        return drivebase.getAutonomousCommand(elasticSubsystem.getSelectedAuto());
    }

    public void setDashboardTab(String name) {
        Elastic.selectTab(name);
    }

    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake); 
    }

    public void registerNamedCommands() {
        NamedCommands.registerCommand("Drive to Test Pose", drivebase.driveToPose(testPose));
    }

    public void sendNotificationsPerodic(){
        if (!DriverStation.isJoystickConnected(Constants.XBOX_PORT)) {
            Notification controllerAlert = new Notification(NotificationLevel.WARNING, "WARNING", "XBOX NOT CONNECTED")
                    .withWidth(600).withAutomaticHeight().withNoAutoDismiss();
            Elastic.sendNotification(controllerAlert);
        }
    }
}