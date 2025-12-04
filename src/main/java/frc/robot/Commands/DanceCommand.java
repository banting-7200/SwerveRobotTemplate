package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.HeadSubsystem;
// import frc.robot.Subsystems.LegSubsystem;
// import frc.robot.Subsystems.LightsSubsystem;

public class DanceCommand extends Command {

  double position = 0;
  double increase = 0.05;
  double time = 0;
  double danceTime = 3.5;

  ArmSubsystem leftArm;
  ArmSubsystem rightArm;
  // LegSubsystem leftLeg;
  // LegSubsystem rightLeg;
  HeadSubsystem head;

  public DanceCommand(RobotContainer robot) {
    leftArm = robot.leftArm;
    rightArm = robot.rightArm;
    // leftLeg = robot.leftLeg;
    // rightLeg = robot.rightLeg;
    head = robot.head;
  }

  @Override
  public void initialize() {
    System.out.println("Starting Dance Command");
  }

  @Override
  public void execute() {
    position += increase;
    time += 0.02;
    if (Math.abs(position) >= 1) {
      increase *= -1;
    }
    leftArm.moveFromRange(-1, 1, position);
    rightArm.moveFromRange(-1, 1, -position);
    // leftLeg.moveFromRange(-2, 1, position);
    // rightLeg.moveFromRange(-2, 1, position);
    head.setHeadPosition(-1, 1, position);
   // lights.rainbow();
  }

  @Override
  public boolean isFinished() {
    return time > danceTime;
  }

  @Override
  public void end(boolean interrupted) {
    leftArm.moveToDownPosition();
    rightArm.moveToDownPosition();
    // rightLeg.moveToUpPosition();
    // leftLeg.moveToUpPosition();
    head.setHeadPosition(-1, 1, -1);
  }
}
