package frc.robot.CameraSystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVisionCamera extends SubsystemBase {
  // private PhotonCamera cam;
  private NetworkTable table;

  public PhotonVisionCamera(String cameraName) {
    table = NetworkTableInstance.getDefault().getTable("photonvision").getSubTable(cameraName);
    // cam = new PhotonCamera(cameraName);
  }

  // @Override
  // public void periodic() {
  //   cam.getall
  // }

  public double getTargetWidth() {
    return (double) table.getEntry("targetPixelsX").getNumber(-1);
  }

  public double getTargetHeight() {
    return (double) table.getEntry("targetPixelsY").getNumber(-1);
  }

  public double getTargetArea() {
    return (double) table.getEntry("targetArea").getNumber(-1);
  }

  public double getTargetYaw() {
    return (double) table.getEntry("targetYaw").getNumber(0);
  }

  public double getTargetPitch() {
    return (double) table.getEntry("targetPitch").getNumber(-10);
  }

  public boolean hasTarget() {
    return table.getEntry("hasTarget").getBoolean(false);
  }

  public void test() {
    System.out.println(
        "Yaw:"
            + String.format("%.2f", getTargetYaw())
            + "Pitch"
            + String.format("%.2f", getTargetPitch()));
  }

  public double getTargetPixelsX() {
    return (double) table.getEntry("targetPixelsX").getNumber(-1);
  }

  public double getTargetPixelsY() {
    return (double) table.getEntry("targetPixelsY").getNumber(-1);
  }

  public double getLatency() {
    return (double) table.getEntry("latencyMillis").getNumber(-1);
  }
}
