package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.SwerveSubsystem;

public class RobotContainer {

    private static RobotContainer instance;
      private static SwerveSubsystem drivebase;
      private Command driveFieldOrientedDirectAngle;
      private XboxController mainController = new XboxController(0);
    private EventLoop loop = new EventLoop();
    
    public static RobotContainer getInstance() {
        if (instance == null) instance = new RobotContainer();
        return instance;
      }


      private RobotContainer(){
       
        drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));
        driveFieldOrientedDirectAngle =
        drivebase.driveCommand(
            () ->
                MathUtil.applyDeadband(
                    -mainController.getLeftY(), 0.1),
            () ->
                MathUtil.applyDeadband(
                    -mainController.getLeftX(), 0.1),
            () -> -mainController.getRightX(),
            () -> -mainController.getRightY());

        swerveConfigBindings();

    
        
    }





    public void swerveConfigBindings() {
    BooleanEvent zeroDriveBase =
        mainController.button(XboxController.Button.kA.value, loop);

    zeroDriveBase.rising().ifHigh(() -> drivebase.zeroGyro());

    BooleanEvent enableCreepDrive =
        mainController.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, 0.5, loop);

    enableCreepDrive.ifHigh(() -> drivebase.setCreepDrive(true));

    drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
  }



    public void teleopPeriodic(){
        loop.poll();
    }






}
