package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;

public record ShootingDataPoint(
    double distance, double shooterSpeed, Rotation2d hoodAngle, Double tof) {
  public ShootingDataPoint(double distance, double shooterSpeed, Rotation2d hoodAngle) {
    this(distance, shooterSpeed, hoodAngle, null);
  }
}
