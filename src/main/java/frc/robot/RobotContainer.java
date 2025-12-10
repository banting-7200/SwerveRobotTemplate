package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.DanceCommand;
import frc.robot.Commands.DriveHorizontalCommand;
import frc.robot.Commands.NetAlignCommand;
import frc.robot.Commands.WaveCommand;
import frc.robot.Constants.Arms;
import frc.robot.Constants.ButtonBoxConfig;
import frc.robot.Constants.CAN_IDS;
import frc.robot.Constants.Legs;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.CameraSystems.DualCameraVelocityTracker;
import frc.robot.CameraSystems.PhotonVisionCamera;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.HeadSubsystem;
import frc.robot.Subsystems.LegSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.Vision;

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

    public HeadSubsystem head;
    public ArmSubsystem leftArm;
    public ArmSubsystem rightArm;
    public LegSubsystem leftLeg;
    public LegSubsystem rightLeg;

    public DualCameraVelocityTracker velocityTracker;
    public PhotonVisionCamera upperCamera;
    public PhotonVisionCamera lowerCamera;
    public PhotonVisionCamera backCamera;

    XboxController driverXbox = new XboxController(0);
    CommandJoystick buttonBox = new CommandJoystick(1);

    // private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
    //         "swerve/neo"));

    private EventLoop enabledLoop = new EventLoop();

    private boolean manualMode = true;
    private boolean flipped = false;
    private boolean useHeadSlider;
    public double secondsBeforeSave;
    public double saveMillis = System.currentTimeMillis();
    public boolean canMakeSave = false;

    /**
     * Converts driver input into a field-relative ChassisSpeeds that is controlled
     * by angular velocity.
     */
    // SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
    //         () -> driverXbox.getLeftY() * -1,
    //         () -> driverXbox.getLeftX() * -1)
    //         .withControllerRotationAxis(driverXbox::getRightX)
    //         .deadband(OperatorConstants.DEADBAND)
    //         .scaleTranslation(0.8)
    //         .allianceRelativeControl(true);

    // /**
    // * Clone's the angular velocity input stream and converts it to a
    // fieldRelative
    // * input stream.
    // */
    // SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
    //         .withControllerHeadingAxis(() -> -driverXbox.getRightX(),
    //                 () -> -driverXbox.getRightY())
    //         .headingWhile(true);

    public RobotContainer() {

        head = new HeadSubsystem();

        leftArm = new ArmSubsystem(
                CAN_IDS.LEFT_ARM_MOTOR,
                Arms.Positions.leftMaxSavePosition,
                Arms.Positions.leftMinSavePosition,
                false);

        leftArm.setPID(Arms.LeftPID.P, Arms.LeftPID.I, Arms.LeftPID.D);

        rightArm = new ArmSubsystem(CAN_IDS.RIGHT_ARM_MOTOR, Arms.Positions.rightMaxSavePosition,
                Arms.Positions.rightMinSavePosition, true);

        rightArm.setPID(Arms.RightPID.P, Arms.RightPID.I, Arms.RightPID.D);

        leftLeg = new LegSubsystem(
                CAN_IDS.LEFT_LEG_MOTOR,
                Legs.Positions.leftUpPosition,
                Legs.Positions.leftDownPosition,
                false);

        leftLeg.setPID(Legs.leftPID.P, Legs.leftPID.I, Legs.leftPID.D);

        rightLeg = new LegSubsystem(
                CAN_IDS.RIGHT_LEG_MOTOR,
                Legs.Positions.rightUpPosition,
                Legs.Positions.rightDownPosition,
                true);

        rightLeg.setPID(Legs.rightPID.P, Legs.rightPID.I, Legs.rightPID.D);

        upperCamera = new PhotonVisionCamera(Constants.Vision.UpperCamera.address);
        lowerCamera = new PhotonVisionCamera(Constants.Vision.LowerCamera.address);
        backCamera = new PhotonVisionCamera(Constants.Vision.BackCamera.address);

        velocityTracker = new DualCameraVelocityTracker(
                upperCamera,
                Constants.Vision.UpperCamera.xOffset,
                Constants.Vision.UpperCamera.yOffset,
                Constants.Vision.UpperCamera.upTilt,
                Constants.Vision.UpperCamera.rightTilt,
                lowerCamera,
                Constants.Vision.LowerCamera.xOffset,
                Constants.Vision.LowerCamera.yOffset,
                Constants.Vision.LowerCamera.upTilt,
                Constants.Vision.LowerCamera.rightTilt);

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

        buttonBox.button(ButtonBoxConfig.zeroHeadButton).onTrue(new InstantCommand(() -> head.reZero()));
        buttonBox.button(ButtonBoxConfig.toggleHeadButton).onTrue(new InstantCommand(() -> head.toggleHead()));

        buttonBox.button(ButtonBoxConfig.leftLegToggleButton)
                .onTrue(new InstantCommand(() -> leftLeg.togglePosition()));
        buttonBox.button(ButtonBoxConfig.rightLegToggleButton)
                .onTrue(new InstantCommand(() -> rightLeg.togglePosition()));

        // buttonBox.button(ButtonBoxConfig.enableAutoSaveButton)
        //         .onTrue(new InstantCommand(() -> ready()));
        // buttonBox.button(ButtonBoxConfig.clearCameraDataButton)
        //         .onTrue(new InstantCommand(() -> reset()).andThen(new NetAlignCommand(drivebase, backCamera)));
        // buttonBox.button(ButtonBoxConfig.invertButtonBoxSwitch).whileTrue(new InstantCommand(() -> flipped = true))
        //         .whileFalse(new InstantCommand(() -> flipped = false));
        // buttonBox.button(ButtonBoxConfig.manualModeSwitch).onTrue(new InstantCommand(() -> setManualMode(false)))
        //         .onFalse(new InstantCommand(() -> setManualMode(true)));

        // buttonBox.button(ButtonBoxConfig.waveButton).onTrue(new WaveCommand(leftArm));
        // buttonBox.button(ButtonBoxConfig.danceButton).onTrue(new DanceCommand(instance));

    }

    public void enabledPerodic() {
        leftArm.setDefaultCommand(
                new RunCommand(
                        () -> leftArm.moveFromRange(-1, 1, -buttonBox.getRawAxis(1)),
                        leftArm));
        leftArm.run();

        rightArm.setDefaultCommand(
                new RunCommand(() -> rightArm.moveFromRange(-1, 1, buttonBox.getRawAxis(0)), rightArm));
        rightArm.run();

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

       // enabledLoop.poll();

       leftLeg.run();
     // rightLeg.run(); // DO NOT RUN UNTIL LIMIT SWITCHES ARE DONE
    }

    // public void estimateHitPoint() {
    //     if (!velocityTracker.hasTarget())
    //         return;
    //     double[] hitPoint = velocityTracker.getHitPoint();
    //     if (hitPoint == new double[2]) {
    //         System.out.println("hasTarget");
    //     } else {
    //         System.out.println(
    //                 String.format(
    //                         "hitpoint: %.2f, %.2f, in %.2f seconds",
    //                         hitPoint[0], hitPoint[1], velocityTracker.getSecondsToImpact()));
    //     }
    // }

    // public void reset() {
    //     velocityTracker.reset();
    //     leftArm.moveToDownPosition();
    //     rightArm.moveToDownPosition();
    //     leftLeg.moveToUpPosition();
    //     rightLeg.moveToUpPosition();
    // }

    // public void ready() {
    //     canMakeSave = true;
    //     // lights.solidColor(255, 0, 0);
    // }

    // public void makeSave() {
    //     if (!canMakeSave) {
    //         // lights.solidColor(0, 255, 0);
    //         return; // ensure it is ready
    //     }
    //     if (velocityTracker.hasTarget()) { // if cameras see the puck
    //         if (velocityTracker.getSecondsToImpact() < Constants.Robot.secondsBeforeSave // if puck going to
    //                 && velocityTracker.getSecondsToImpact() > 0) {
    //             double saveTime = -Timer.getFPGATimestamp();
    //             canMakeSave = false;
    //             // lights.solidColor(0, 255, 0);
    //             System.out.println("Tracker Latency: " + velocityTracker.getLatency());
    //             double[] hitPoint = velocityTracker.getHitPoint();
    //             if (hitPoint[1] > Constants.Robot.legActivationMaxHeight) { // if not legs
    //                 if (hitPoint[1] < Constants.Robot.armActivationMaxHeight) { // if not above net
    //                     // arms
    //                     if (hitPoint[0] > Constants.Robot.width / 2) { // if on right
    //                         if (hitPoint[1] > Constants.Robot.rightArmActivationMinHeight) { // if within arm range on
    //                                                                                          // right
    //                             rightArmSave(hitPoint[1]);
    //                         } else { // if in between arm and leg on right
    //                             rightMiddleSave();
    //                         }
    //                     } else if (hitPoint[0] < -Constants.Robot.width / 2) { // if on left
    //                         if (hitPoint[1] > Constants.Robot.leftArmActivationMinHeight) { // if within arm range on
    //                                                                                         // right
    //                             leftArmSave(hitPoint[1]);
    //                         } else {
    //                             leftMiddleSave();
    //                         }
    //                     } else { // if in middle
    //                         System.out.println("Torso");
    //                     }
    //                 } else { // if above net
    //                     System.out.println("Too High");
    //                     leftArm.moveToUpPosition();
    //                     rightArm.moveToUpPosition();
    //                 }
    //             } else { // if legs
    //                 if (hitPoint[0] > Constants.Robot.width / 2) { // if on right
    //                     rightLegSave();
    //                 } else if (hitPoint[0] < -Constants.Robot.width / 2) { // if on left
    //                     leftLegSave();
    //                 } else { // if in middle
    //                     middleLegSave();
    //                 }
    //             }
    //             System.out.println(String.format("Hitpoint: %2f, %2f", hitPoint[0], hitPoint[1]));
    //             saveTime += Timer.getFPGATimestamp();
    //             System.out.println("Save Time: " + saveTime);
    //         } else {
    //             System.out.println("Has Target " + velocityTracker.getSecondsToImpact());
    //         }
    //     }
    // }

    // public void leftLegSave() {
    //     leftLeg.moveToDownPosition();
    //     rightLeg.moveToMidPosition();
    //     System.out.println("Left Leg");
    //     new DriveHorizontalCommand(drivebase, -Constants.Robot.SlideDistance).schedule();
    // }

    // public void rightLegSave() {
    //     rightLeg.moveToDownPosition();
    //     leftLeg.moveToMidPosition();
    //     System.out.println("Right Leg");
    //     new DriveHorizontalCommand(drivebase, Constants.Robot.SlideDistance).schedule();
    // }

    // public void leftArmSave(double height) {
    //     double armPercent = ((height - Constants.Robot.leftArmActivationMinHeight)
    //             / (Constants.Robot.leftArmActivationMaxHeight
    //                     - Constants.Robot.leftArmActivationMinHeight));
    //     leftArm.moveFromRange(0, 0.8, armPercent);
    //     rightLeg.moveToMidPosition();
    //     System.out.println("Left Arm");
    //     new DriveHorizontalCommand(drivebase, -Constants.Robot.SlideDistance).schedule();
    // }

    // public void rightArmSave(double height) {
    //     double armPercent = ((height - Constants.Robot.rightArmActivationMinHeight)
    //             / (Constants.Robot.rightArmActivationMaxHeight
    //                     - Constants.Robot.rightArmActivationMinHeight));
    //     rightArm.moveFromRange(0, 0.8, armPercent);
    //     leftLeg.moveToMidPosition();
    //     System.out.println("Right Arm");
    //     new DriveHorizontalCommand(drivebase, Constants.Robot.SlideDistance).schedule();
    // }

    // public void rightMiddleSave() {
    //     rightArm.moveToDownPosition();
    //     rightLeg.moveToUpPosition();
    //     leftLeg.moveToMidPosition();
    //     new DriveHorizontalCommand(drivebase, 1.5 * Constants.Robot.SlideDistance).schedule();
    //     System.out.println("Right Middle");
    // }

    // public void leftMiddleSave() {
    //     rightArm.moveToDownPosition();
    //     leftLeg.moveToUpPosition();
    //     rightLeg.moveToMidPosition();
    //     new DriveHorizontalCommand(drivebase, -1.5 * Constants.Robot.SlideDistance).schedule();
    //     System.out.println("Left Middle");
    // }

    // public void middleLegSave() {
    //     rightLeg.moveToDownPosition();
    //     leftLeg.moveToDownPosition();
    //     System.out.println("Both Legs");
    // }

    // public void countFrames() {
    //     if (velocityTracker.hasTarget()) {
    //         if (velocityTracker.getSecondsToImpact() < Constants.Robot.secondsBeforeSave
    //                 && velocityTracker.getSecondsToImpact() > 0) {
    //             System.out.println(velocityTracker.getSecondsToImpact());
    //         } else {
    //             System.out.println("hasTarget");
    //         }
    //     }
    // }

    // public void setManualMode(boolean manualMode) {
    //     canMakeSave = false;
    //     this.manualMode = manualMode;
    // }

}