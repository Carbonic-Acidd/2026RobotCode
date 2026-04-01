// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import au.grapplerobotics.LaserCan;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SpindexerConstants;

public class Spindexer extends SubsystemBase {
  private TalonFX spindexerMotor;
  private TalonFX kickerMotor;
  private Debouncer currentEmptyDebouncer = new Debouncer(1.0);

  private int ballCounter = 0;

  private LaserCan kickerLaser;

  private StatusSignal<Current> kickerCurrent;
  private StatusSignal<Current> spindexerCurrent;

  /** Creates a new Spindexer. */
  public Spindexer() {
    spindexerMotor = new TalonFX(SpindexerConstants.spindexerMotorID);
    kickerMotor = new TalonFX(SpindexerConstants.kickerMotorID);
    kickerLaser = new LaserCan(SpindexerConstants.kickerLaserID);

    spindexerMotor.getConfigurator().apply(SpindexerConstants.spindexerConfigs);
    kickerMotor.getConfigurator().apply(SpindexerConstants.spindexerConfigs);

    kickerCurrent = kickerMotor.getStatorCurrent();
    spindexerCurrent = spindexerMotor.getStatorCurrent();
  }

  public void addBall() {
    ballCounter++;
  }

  public void zeroBalls() {
    ballCounter = 0;
  }

  public int getBalls() {
    return ballCounter;
  }

  public boolean kickerLaserBroken() {
    LaserCan.Measurement measurement = kickerLaser.getMeasurement();

    if (measurement != null
        && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT
        && measurement.distance_mm < 45) {
      return true;
    } else {
      return false;
    }
  }

  public void stopSpindexerMotor() {
    spindexerMotor.stopMotor();
  }

  public void stopKickerMotor() {
    kickerMotor.stopMotor();
  }

  public double getSpindexerSpeed() {
    return spindexerMotor.get();
  }

  public double getKickerSpeed() {
    return kickerMotor.get();
  }

  public boolean currentSaysEmpty() {
    return currentEmptyDebouncer.calculate(
        spindexerCurrent.getValue().in(Amps) < 9.0); // random number need to test
  }

  public void decide(double startTime) {
    if (Timer.getFPGATimestamp() - startTime > 1.5 || DriverStation.isAutonomous()) {
      spindexerMotor.stopMotor();
      kickerMotor.set(SpindexerConstants.kickerMotorSpeed);

    } else {
      spindexerMotor.stopMotor();
      kickerMotor.stopMotor();
    }
  }

  public void runBoth() {
    spindexerMotor.set(SpindexerConstants.spindexerMotorSpeed);
    kickerMotor.set(SpindexerConstants.kickerMotorSpeed);
  }

  public void stopBoth() {
    spindexerMotor.stopMotor();
    kickerMotor.stopMotor();
  }

  public Command idleReverse() {
    return run(
        () -> {
          spindexerMotor.set(SpindexerConstants.spindexerIdleSpeed);
          kickerMotor.set(SpindexerConstants.kickerIdleSpeed);
        });
  }

  public Command manualBoth() {
    return run(
        () -> {
          spindexerMotor.set(SpindexerConstants.spindexerMotorSpeed);
          kickerMotor.set(SpindexerConstants.kickerMotorSpeed);
        });
  }

  public Command manualReverse() {
    return run(
        () -> {
          spindexerMotor.set(-SpindexerConstants.spindexerMotorSpeed);
          kickerMotor.set(-SpindexerConstants.kickerMotorSpeed);
        });
  }

  public Command runSpindexer() {
    return run(() -> spindexerMotor.set(SpindexerConstants.spindexerMotorSpeed));
  }

  public Command runKicker() {
    return run(() -> kickerMotor.set(SpindexerConstants.kickerMotorSpeed));
  }

  public Command stopSpindexerCommand() {
    return runOnce(this::stopSpindexerMotor);
  }

  public Command stopKickerCommand() {
    return runOnce(this::stopKickerMotor);
  }

  public Command runUntilEmptyCommand() {
    return (run(() -> runBoth())).until(() -> currentSaysEmpty());
  }

  public boolean isEmpty() {
    return (spindexerMotor.get() > 0.1) && currentSaysEmpty();
  }

  @Override
  public void periodic() {
    spindexerCurrent.refresh();
    kickerCurrent.refresh();

    SmartDashboard.putNumber("Spindexer/Ball Counter", ballCounter);
    SmartDashboard.putBoolean("Spindexer/Kicker Laser Broken", kickerLaserBroken());
    SmartDashboard.putNumber("Spindexer/Spindexer Current", spindexerCurrent.getValue().in(Amps));
    SmartDashboard.putNumber("Spindexer/Kicker Current", kickerCurrent.getValue().in(Amps));
    SmartDashboard.putBoolean("Spindexer/Spindexer Empty", isEmpty());
  }
}
