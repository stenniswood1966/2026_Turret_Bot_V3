// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.constants;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.Supplier;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

public class HoodSubsystem extends SubsystemBase {
  private static TalonFX motor1 = new TalonFX(constants.kg_HoodSubsystem.k_Motor1ID, constants.kg_HoodSubsystem.k_Motor1CANBus);//change me
  private MotionMagicVoltage mmReq = new MotionMagicVoltage(0);

  private final StatusSignal<Angle> motor1MotorPosition= motor1.getPosition(false);
  private static Boolean homeFlag = true;
  private Boolean killFlag = false;

  private final DutyCycleOut calibrationRequest = new DutyCycleOut(constants.kg_HoodSubsystem.k_HomePower)
    .withIgnoreHardwareLimits(true)
    .withIgnoreSoftwareLimits(true);

  /** Creates a new HoodSubsystem. */
  public HoodSubsystem() {
    //configure the TalonFX motors
    var fx_cfg = new TalonFXConfiguration(); //creates a default TalonFX configuration

    //configuration for motor1
    fx_cfg.MotorOutput.Inverted = constants.kg_HoodSubsystem.k_Motor1InvertedValue;
    fx_cfg.MotorOutput.NeutralMode = constants.kg_HoodSubsystem.k_Motor1NeutralModeValue;
    fx_cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = constants.kg_HoodSubsystem.k_Motor1ForwardSoftLimitEnable;
    fx_cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = constants.kg_HoodSubsystem.k_Motor1ReverseSoftLimitEnable;
    fx_cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.kg_HoodSubsystem.k_Motor1ForwardSoftLimitThreshold;
    fx_cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants.kg_HoodSubsystem.k_Motor1ReverseSoftLimitThreshold;

    //amp limits
    fx_cfg.CurrentLimits.StatorCurrentLimit = constants.kg_HoodSubsystem.k_Motor1StatorCurrentLimit;
    fx_cfg.CurrentLimits.StatorCurrentLimitEnable = constants.kg_HoodSubsystem.k_Motor1StatorCurrentLimitEnable;
    fx_cfg.CurrentLimits.SupplyCurrentLimit = constants.kg_HoodSubsystem.k_Motor1SupplyCurrentLimit;
    fx_cfg.CurrentLimits.SupplyCurrentLimitEnable = constants.kg_HoodSubsystem.k_Motor1StatorCurrentLimitEnable;
    fx_cfg.CurrentLimits.SupplyCurrentLowerLimit = constants.kg_HoodSubsystem.k_Motor1SupplyCurrentLowerLimit;
    fx_cfg.CurrentLimits.SupplyCurrentLowerTime = constants.kg_HoodSubsystem.k_Motor1SupplyCurrentLowerTime;

    // set slot 0 gains
    var slot0Configs = fx_cfg.Slot0;
    slot0Configs.kS = constants.kg_HoodSubsystem.k_Motor1kS; // Add 0.25 V output to overcome static friction change me
    slot0Configs.kV = constants.kg_HoodSubsystem.k_Motor1kV; // A velocity target of 1 rps results in 0.12 V output change me
    slot0Configs.kA = constants.kg_HoodSubsystem.k_Motor1kA; // An acceleration of 1 rps/s requires 0.01 V output change me
    slot0Configs.kP = constants.kg_HoodSubsystem.k_Motor1kP; // A position error of 1.5 rotation results in 12 V output change me
    slot0Configs.kI = constants.kg_HoodSubsystem.k_Motor1kI; // no output for integrated error change me
    slot0Configs.kD = constants.kg_HoodSubsystem.k_Motor1kD; // A velocity error of 1 rps results in 0.1 V output change me
    slot0Configs.StaticFeedforwardSign = constants.kg_HoodSubsystem.k_Motor1StaticFeedforwardSign;
    slot0Configs.GravityType = constants.kg_HoodSubsystem.k_Motor1GravityTypeValue;

    //set Motion Magic Settings
    var motionMagicConfigs = fx_cfg.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = constants.kg_HoodSubsystem.k_Motor1MotionMagicCruiseVelocity; // Target cruise velocity of 90 rps change me
    motionMagicConfigs.MotionMagicAcceleration = constants.kg_HoodSubsystem.k_Motor1MotionMagicAcceleration; // Target acceleration of 160 rps/s (0.5 seconds) change me
    motionMagicConfigs.MotionMagicJerk = constants.kg_HoodSubsystem.k_Motor1MotionMagicJerk; // Target jerk of 1600 rps/s/s (0.1 seconds); change me

    motor1.getConfigurator().apply(fx_cfg, 0.050); //apply configuration to motor
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(
      motor1MotorPosition
    );
  }
    
  public void enablemotionmagic(double targetpos) {
    // periodic, run Motion Magi with slot 0 configs,
    if (!killFlag && !homeFlag) {
      motor1.setControl(mmReq.withPosition(targetpos).withSlot(0));
    }
  }

  public Command stopCommand(){
    return runOnce(() -> stop());
  }

  private void stop()
  {
    motor1.set(0);
  }

  //this method allows us to get the position of motor 1. This is important because we need to force the command that calls 
    //this to update the position 
  public double getPosition()
  {
    return Math.abs(motor1.getPosition().getValueAsDouble());
  }

  public Trigger t_hoodInRange(Angle threshold){
    return new Trigger(() -> {
      return motor1MotorPosition.isNear(Angle.ofRelativeUnits(mmReq.Position, Rotations), threshold);
    });
  }

  public Command setTarget(Supplier<Double> target){
    return runOnce(() -> {
      double t = target.get();
      enablemotionmagic(t);
    });
  }

  public Command setHome(Boolean state){
    return runOnce(() -> {
      setHomeMethod(state);
    });
  }

  private void setHomeMethod(Boolean state){
    enablemotionmagic(constants.kg_HoodSubsystem.k_HomePosition);
    homeFlag = state;
  }

  public Command killCommand(){
    return runOnce(() -> killMethod());
  }

  private void killMethod(){
    stop();
    killFlag = !killFlag;
  }
  
  public final Trigger isHardStop = new Trigger(() -> {
    return motor1.getVelocity().getValue().abs(RotationsPerSecond) < 1 &&
      motor1.getTorqueCurrent().getValue().abs(Amps) > 31;
  }).debounce(0.1);

  public Command homeCommand(){
    return run(() -> {
      motor1.setControl(calibrationRequest);
    })
    .until(isHardStop)
    .andThen(stopCommand()
    .finallyDo(() -> {
      motor1.setPosition(0);
    }));
  }

  public Command setInCommand(){
    return runOnce(() -> setInMethod()); 
  }

  private void setInMethod(){
    if (!killFlag) {
      enablemotionmagic(constants.kg_HoodSubsystem.k_HomePosition);
    }
  }

  public Command setOutCommand(){
    return runOnce(() -> setOutMethod()); 
  }

  private void setOutMethod(){
    if (!killFlag) {
      enablemotionmagic(17);
    }
  }
}