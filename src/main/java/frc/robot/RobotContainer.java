package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Arms;
import frc.robot.Constants.ButtonBoxConfig;
import frc.robot.Constants.CAN_IDS;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.HeadSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.WaveCommand;

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

    public ArmSubsystem leftArm;
    public ArmSubsystem rightArm;

    public HeadSubsystem head;
    // final CommandXboxController driverXbox = new CommandXboxController(0);
    XboxController controller = new XboxController(0);
    Joystick buttonBox = new Joystick(1);
    JoystickButton waveButton = new JoystickButton(buttonBox, ButtonBoxConfig.enableMotorsSwitch);

    // private final SwerveSubsystem drivebase = new SwerveSubsystem(new
    // File(Filesystem.getDeployDirectory(),
    // "swerve/neo"));

    private EventLoop enabledLoop = new EventLoop();

    private boolean useHeadSlider;
    private boolean isWaving = false;

    /**
     * Converts driver input into a field-relative ChassisSpeeds that is controlled
     * by angular velocity.
     */
    // SwerveInputStream driveAngularVelocity =
    // SwerveInputStream.of(drivebase.getSwerveDrive(),
    // () -> driverXbox.getLeftY() * -1,
    // () -> driverXbox.getLeftX() * -1)
    // .withControllerRotationAxis(driverXbox::getRightX)
    // .deadband(OperatorConstants.DEADBAND)
    // .scaleTranslation(0.8)
    // .allianceRelativeControl(true);

    // /**
    // * Clone's the angular velocity input stream and converts it to a
    // fieldRelative
    // * input stream.
    // */
    // SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
    // .withControllerHeadingAxis(() -> -driverXbox.getRightX(),
    // () -> -driverXbox.getRightY())
    // .headingWhile(true);

    public RobotContainer() {

        // WaveCommand waveCommand = new WaveCommand(leftArm);
        head = new HeadSubsystem();
        // NamedCommands.registerCommand("test", Commands.print("I EXIST"));

        leftArm = new ArmSubsystem(
                CAN_IDS.LEFT_ARM_MOTOR,
                Arms.Positions.leftMaxSavePosition,
                Arms.Positions.leftMinSavePosition,
                false);

        leftArm.setPID(Arms.LeftPID.P, Arms.LeftPID.I, Arms.LeftPID.D);

        configureBindings();
    }

    private void configureBindings() {
        // Command driveFieldOrientedDirectAngle =
        // drivebase.driveFieldOriented(driveDirectAngle); // What type of drive

        // drivebase.setDefaultCommand(driveFieldOrientedDirectAngle); // this line is
        // the actual "drive"

        // driverXbox.rightTrigger(0.2).whileTrue(Commands.runOnce(() ->
        // drivebase.setCreepDrive(true)).repeatedly());
        // driverXbox.rightTrigger(0.2).whileFalse(Commands.runOnce(() ->
        // drivebase.setCreepDrive(false)).repeatedly());

        BooleanEvent toggleHead = new BooleanEvent(
                enabledLoop, () -> buttonBox.getRawButton(ButtonBoxConfig.toggleHeadButton));

        toggleHead.and(() -> !useHeadSlider).rising().ifHigh(() -> head.toggleHead());

        BooleanEvent zeroHead = new BooleanEvent(enabledLoop,
                () -> buttonBox.getRawButton(ButtonBoxConfig.zeroHeadButton));
        zeroHead.rising().ifHigh(() -> head.reZero());

        waveButton.whileTrue((new WaveCommand(leftArm)).repeatedly());

    }

    public void enabledPerodic() {
        leftArm.setDefaultCommand(
                new RunCommand(
                        () -> leftArm.moveFromRange(0, 1, -controller.getLeftY()),
                        leftArm));

        leftArm.run();

        head.zeroEncoderPeriodic();
        head.run();
        if (buttonBox.getRawAxis(2) > 0.9) {
            useHeadSlider = false;
        } else {
            useHeadSlider = true;
        }
        if (useHeadSlider) {
            head.setHeadPosition(1, -1, buttonBox.getRawAxis(2));
        }

        enabledLoop.poll();

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    // public Command getAutonomousCommand() {
    // // An example command will be run in autonomous
    // return drivebase.getAutonomousCommand("New Auto");
    // }

    // public void setMotorBrake(boolean brake) {
    // drivebase.setMotorBrake(brake);
    // }
}