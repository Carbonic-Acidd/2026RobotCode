package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private TalonFX shooterMotor = new TalonFX(ShooterConstants.shooterMotorID);

  private final MotionMagicVelocityVoltage velocityMMRequest = new MotionMagicVelocityVoltage(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
  private LinearVelocity goalSpeed = MetersPerSecond.of(0);

  public Shooter() {
    shooterMotor.getConfigurator().apply(ShooterConstants.shooterConfigs);
  }

  public void setGoalSpeed(LinearVelocity speed) {
    goalSpeed = speed;
  }

  public LinearVelocity getGoalSpeed() {
    return goalSpeed;
  }

  public void stop() {
    // shooterMotor.stopMotor();
    goalSpeed = MetersPerSecond.of(0);
  }

  public AngularVelocity linearToAngularVelocity(LinearVelocity vel) {
    return RadiansPerSecond.of(
        vel.in(MetersPerSecond) / ShooterConstants.flyWheelRadius.in(Meters));
  }

  public LinearVelocity angularToLinearVelocity(AngularVelocity vel) {
    return MetersPerSecond.of(
        vel.in(RadiansPerSecond) * ShooterConstants.flyWheelRadius.in(Meters));
  }

  public AngularVelocity getCurrentVelocity() {
    return shooterMotor.getVelocity().getValue();
  }

  public boolean shooterAtSetPoint() {
    LinearVelocity currentSpeed = angularToLinearVelocity(getCurrentVelocity());

    return Math.abs(currentSpeed.in(MetersPerSecond) - goalSpeed.in(MetersPerSecond))
        < ShooterConstants.shooterSpeedTolerance.in(MetersPerSecond);
    // return true;
  }

  public Command runMotor() {
    return run(() -> shooterMotor.set(0.40)); // .45
  }

  public Command stopMotor() {
    return runOnce(() -> shooterMotor.set(0.0));
  }

  // public void logSolution(ShotSolution solution) {
  //   currentShotSolution = solution;
  //   setGoalSpeed(solution.exitVelocity());
  // }

  // public ShotSolution getShotSolution() {
  //   return currentShotSolution;
  // }

  // public LinearVelocity getExitVelocity() {
  //   return MetersPerSecond.of(9.353);
  // }

  @Override
  public void periodic() {
    shooterMotor.setControl(velocityRequest.withVelocity(linearToAngularVelocity(goalSpeed)));

    // shooterMotor.setControl(
    //     velocityRequest.withVelocity(
    //         linearToAngularVelocity(
    //             MetersPerSecond.of(SmartDashboard.getNumber("Dynamic Shooter Speed", 0)))));
    // shooterMotor.set(SmartDashboard.getNumber("Dynamic Shooter Speed", 0));
    SmartDashboard.putNumber(
        "Shooter/Current Angular Velocity", getCurrentVelocity().in(RotationsPerSecond));
    SmartDashboard.putNumber(
        "Shooter/Current Linear Velocity",
        angularToLinearVelocity(getCurrentVelocity()).in(MetersPerSecond));
    SmartDashboard.putNumber(
        "Shooter/Goal Angular Velocity", linearToAngularVelocity(goalSpeed).in(RotationsPerSecond));
    // SmartDashboard.putNumber("Shooter/RPM",
    // (angularToLinearVelocity(getCurrentVelocity())/(Math.PI*))
    SmartDashboard.putNumber("Shooter/Goal Linear Velocity", goalSpeed.in(MetersPerSecond));
  }
}
