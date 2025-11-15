package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.Command;

public class WaveCommand extends Command {

  double position = 0;
  double increase = 0.05;
  double time = 0;
  double waveTime = 3;

  ArmSubsystem arm;

  public WaveCommand(ArmSubsystem arm) {
    this.arm = arm;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    System.out.println("Starting Wave Command");
  }

  @Override
  public void execute() {
    position += increase;
    if (Math.abs(position) >= 1) {
      increase *= -1;
    }
    time += 0.02;
    arm.moveFromRangeWave(-1, 1, position);
  }

  @Override
  public boolean isFinished() {
    return time > waveTime;
  }

  @Override
  public void end(boolean interrupted) {
    arm.moveToDownPosition();
  }
}
