// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.*;

public class Pigeon2Subsystem extends SubsystemBase {

  private final Pigeon2 pigeon2 = new Pigeon2(TunerConstants.kPigeonId, TunerConstants.kCANBus);

  private StatusSignal<Angle> yawPosition;
  private StatusSignal<AngularVelocity> yawVelocity;
  private StatusSignal<Angle> pitchPosition;
  private StatusSignal<Angle> rollPosition;
  private BaseStatusSignal[] signals;

  public Pigeon2Subsystem() {
    pigeon2.getConfigurator().apply(new Pigeon2Configuration());
    var pigeon2Configs = new Pigeon2Configuration();
    pigeon2Configs.MountPose.MountPosePitch = 0.7112835645675659;
    pigeon2Configs.MountPose.MountPoseRoll = -0.9117757081985474;
    pigeon2Configs.MountPose.MountPoseYaw = -85.36386108398438;
    pigeon2.getConfigurator().apply(pigeon2Configs);

    yawPosition = pigeon2.getYaw();
    yawVelocity = pigeon2.getAngularVelocityZDevice();
    pitchPosition = pigeon2.getPitch();
    rollPosition = pigeon2.getRoll();

    pigeon2.setYaw(0);

    signals = new BaseStatusSignal[2];
    signals[0] = yawPosition;
    signals[1] = yawVelocity;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pitch", getPigeonPitch());
    SmartDashboard.putNumber("Roll", getPigeonRoll());
    SmartDashboard.putNumber("Yaw", getPigeonYaw(true));
    SmartDashboard.putNumber("DegreeRot", getGyroRotation(false).getDegrees());
}

  /**
   * getGyroRotation - this is the Yaw value (rotate around...)
   * @param refresh Is new data needed or has the data already been updated this loop
   * @return Rotation2d
   */
  public Rotation2d getGyroRotation(boolean refresh) {
    if(refresh) {
      yawPosition.refresh();
      yawVelocity.refresh();
    }
    
    Measure<AngleUnit> yawRotation = BaseStatusSignal.getLatencyCompensatedValue(yawPosition, yawVelocity);
    return Rotation2d.fromDegrees(yawRotation.baseUnitMagnitude());
  }

  /**
   * getPigeonPitch - this is the pitch value (tilt up or down)
   * @return double
   */
  public double getPigeonPitch(){
    pitchPosition.refresh();
    return pitchPosition.getValue().baseUnitMagnitude();
  }

  /**
   * getPigeonRoll - this is the roll value (tilt left or right)
   * @return double
   */
  public double getPigeonRoll(){
    rollPosition.refresh();
    return rollPosition.getValue().baseUnitMagnitude();
  }

  /**
   * getPigeonYaw - this is the Yaw value (rotate around...)
   * @param refresh Is new data needed or has the data already been updated this loop
   * @return double representing angle in degrees
   */
  public double getPigeonYaw(boolean refresh){
    if(refresh){
      yawPosition.refresh();
      yawVelocity.refresh();
    }

    Measure<AngleUnit> yawRotation = BaseStatusSignal.getLatencyCompensatedValue(yawPosition, yawVelocity);
    return yawRotation.baseUnitMagnitude();
  }

  /**
   * getSignals - signals for pigeon yaw and yaw angular velocity. This gives control of the updates and is used for synchronous updates
   * @return BaseStatusSignal[] of yaw and yaw angular velocity
   */
  public BaseStatusSignal[] getSignals() {
    return signals;
  }
}