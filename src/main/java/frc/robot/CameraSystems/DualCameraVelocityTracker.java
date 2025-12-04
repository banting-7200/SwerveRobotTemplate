package frc.robot.CameraSystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.ArrayList;

public class DualCameraVelocityTracker extends SubsystemBase {

  private final PhotonVisionCamera c1;
  private final PhotonVisionCamera c2;

  private final double c1xOffset;
  private final double c1yOffset;

  private final double c2xOffset;
  private final double c2yOffset;

  private final double c1UpTilt;
  private final double c2UpTilt;

  private final double c1RightTilt;
  private final double c2RightTilt;

  private final double frameRate = 120;
  private final double minFrames = 2; // minimum frames needed to determine average velocities

  private ArrayList<double[]> positions = new ArrayList<>();
  private ArrayList<double[]> velocities = new ArrayList<>();

  /**
   * Initializes a dual camera velocity tracker object.
   *
   * @param camera1 the upper camera
   * @param camera1xOffset the x offset of this camera from the centre of the robot in metres
   * @param camera1yOffset the y offset of this camera from the centre of the robot in metres
   * @param camera1Tilt the upward tilt of this camera in degrees
   * @param camera2 the lower camera
   * @param camera2xOffset the x offset of this camera from the centre of the robot in metres
   * @param camera2yOffset the y offset of this camera from the centre of the robot in metres
   * @param camera2Tilt the upward tilt of this camera in degrees
   */
  public DualCameraVelocityTracker(
      PhotonVisionCamera camera1,
      double camera1xOffset,
      double camera1yOffset,
      double camera1UpTilt,
      double camera1RightTilt,
      PhotonVisionCamera camera2,
      double camera2xOffset,
      double camera2yOffset,
      double camera2UpTilt,
      double camera2RightTilt) {
    this.c1 = camera1;
    this.c2 = camera2;
    this.c1xOffset = camera1xOffset;
    this.c1yOffset = camera1yOffset;
    this.c2xOffset = camera2xOffset;
    this.c2yOffset = camera2yOffset;
    this.c1UpTilt = camera1UpTilt;
    this.c2UpTilt = camera2UpTilt;
    this.c1RightTilt = camera1RightTilt;
    this.c2RightTilt = camera2RightTilt;
  }

  @Override
  public void periodic() {
    if (hasTarget()) {
      addRecentPosition();
      addRecentVelocity();
    }
  }

  public double getDistance() {
    double verticalGap = Math.abs(c1yOffset - c2yOffset);

    double pitch1 = Math.toRadians(Math.abs(90 + c1.getTargetPitch() - c1UpTilt));
    double pitch2 = Math.toRadians(Math.abs(90 - c2.getTargetPitch() - c2UpTilt));
    double oppPitch = Math.PI - pitch1 - pitch2;

    double pSide1 = Math.sin(pitch2) * verticalGap / Math.sin(oppPitch);
    double pSide2 = Math.sin(pitch1) * verticalGap / Math.sin(oppPitch);

    double pSemiPerimeter = (pSide1 + pSide2 + verticalGap) / 2;

    double pArea =
        Math.sqrt(
            pSemiPerimeter
                * (pSemiPerimeter - verticalGap)
                * (pSemiPerimeter - pSide1)
                * (pSemiPerimeter - pSide2));

    double vDistanceEstimate = pArea / (2 * verticalGap);

    return vDistanceEstimate;
  }

  public double getTargetXPosition() {
    return (getDistance() / Math.tan(Math.toRadians(90 - c1.getTargetYaw() - c1RightTilt)))
        + c1xOffset;
  }

  public double getTargetYPosition() {
    return (-getDistance() / Math.tan(Math.toRadians(90 + c1.getTargetPitch() - c1UpTilt)))
        + c1yOffset / 2;
  }

  public void addPosition(double x, double y, double z) {
    double[] position = {x, y, z};
    positions.add(position);
  }

  public void addRecentPosition() {
    addPosition(getTargetXPosition(), getTargetYPosition(), getDistance());
  }

  public Double getRecentHorizontalVelocity() {
    if (positions.size() < 2) return null;
    double position1 = positions.get(positions.size() - 2)[0];
    double position2 = positions.get(positions.size() - 1)[0];
    double velocity = (position2 - position1) * frameRate;
    return velocity;
  }

  public Double getRecentVerticalVelocity() {
    if (positions.size() < 2) return null;
    double position1 = positions.get(positions.size() - 2)[1];
    double position2 = positions.get(positions.size() - 1)[1];
    double velocity = (position2 - position1) * frameRate;
    return velocity;
  }

  public Double getRecentIncomingVelocity() {
    if (positions.size() < 2) return null;
    double position1 = positions.get(positions.size() - 2)[2];
    double position2 = positions.get(positions.size() - 1)[2];
    double velocity = (position1 - position2) * frameRate;
    return velocity;
  }

  public void addRecentVelocity() {
    if (positions.size() < 2) return;
    double[] velocity = {
      getRecentHorizontalVelocity(), getRecentVerticalVelocity(), getRecentIncomingVelocity()
    };
    velocities.add(velocity);
  }

  public double getAverageHorizontalVelocity() {
    double avgVelocity = 0;
    for (double[] velocity : velocities) {
      avgVelocity += velocity[0];
    }
    avgVelocity /= velocities.size();
    return avgVelocity;
  }

  public double getAverageVerticalVelocity() {
    double avgVelocity = 0;
    for (double[] velocity : velocities) {
      avgVelocity += velocity[1];
    }
    avgVelocity /= velocities.size();
    return avgVelocity;
  }

  public double getAverageIncomingVelocity() {
    double avgVelocity = 0;
    for (double[] velocity : velocities) {
      avgVelocity += velocity[2];
    }
    avgVelocity /= velocities.size();
    return avgVelocity;
  }

  public double getRecentAverageHorizontalVelocity() {
    double avgVelocity = 0;
    if (velocities.size() < minFrames) return 0;
    for (int i = 0; i < minFrames; i++) {
      avgVelocity += velocities.get(velocities.size() - i - 1)[0];
    }
    avgVelocity /= minFrames;
    return avgVelocity;
  }

  public double getRecentAverageVerticalVelocity() {
    double avgVelocity = 0;
    if (velocities.size() < minFrames) return 0;
    for (int i = 0; i < minFrames; i++) {
      avgVelocity += velocities.get(velocities.size() - i - 1)[1];
    }
    avgVelocity /= minFrames;
    return avgVelocity;
  }

  public double getRecentAverageIncomingVelocity() {
    double avgVelocity = 0;
    if (velocities.size() < minFrames) return 0;
    for (int i = 0; i < minFrames; i++) {
      avgVelocity += velocities.get(velocities.size() - i - 1)[2];
    }
    avgVelocity /= minFrames;
    return avgVelocity;
  }

  public double getSecondsToImpact() {
    if (velocities.size() < minFrames) return 100;
    return getDistance() / getRecentAverageIncomingVelocity();
  }

  public double[] getHitPoint() {
    double[] hitPoint = new double[2];
    if (velocities.size() < minFrames) return hitPoint;

    double secondsToImpact = getSecondsToImpact();
    hitPoint[0] = getTargetXPosition() + (getRecentAverageHorizontalVelocity() * secondsToImpact);
    // hitPoint[1] = getTargetYPosition() + (getRecentAverageVerticalVelocity() * secondsToImpact)
    //     - (Constants.Robot.GravityEffect * secondsToImpact);
    hitPoint[1] =
        getTargetYPosition()
            + (getRecentAverageVerticalVelocity() * secondsToImpact)
            - (Constants.Robot.GravityEffect * secondsToImpact * secondsToImpact / 2);
    return hitPoint;
  }

  /**
   * Prints test data about the position of the target based on the current frame, regardless of
   * motion. - distance in metres - position right in metres - position up in metres
   */
  public void stationaryTest() {
    if (!hasTarget()) return;
    System.out.println(
        "Distance:"
            + String.format("%.2f", getDistance())
            + " Right:"
            + String.format("%.2f", getTargetXPosition())
            + " Up:"
            + String.format("%.2f", getTargetYPosition()));
  }

  /**
   * Prints test data about the velocity and trajectory of the target in last 20 frames. - incoming
   * velocity in metres per second - rightward velocity in metres per second - upward velocity in
   * metres per second
   */
  public void trajectoryTest() {
    if (!hasTarget()) return;
    System.out.println(
        "Incoming:"
            + String.format("%.2f", getRecentAverageIncomingVelocity())
            + " Horizontal:"
            + String.format("%.2f", getRecentAverageHorizontalVelocity())
            + " Vertical:"
            + String.format("%.2f", getRecentAverageVerticalVelocity()));
  }

  public void reset() {
    positions.clear();
    velocities.clear();
  }

  public boolean hasTarget() {
    return (c1.hasTarget() && c2.hasTarget());
  }

  public double getLatency() {
    double latency1 = c1.getLatency();
    double latency2 = c2.getLatency();
    return (latency1 > latency2 ? latency1 : latency2);
  }
}
