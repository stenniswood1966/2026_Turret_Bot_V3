// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants;

public class DistanceAndAngleCalcSubsystem extends SubsystemBase {
  CommandSwerveDrivetrain drivetrain;
  HoodSubsystem hoodsubsystem;
  ShooterSubsystem shootersubsystem;
  TurretSubsystem turretsubsystem;

  //calculator stuff
  SwerveDriveState state;
  Pose2d robotPose;

  Pose2d targetPose;
  
  double mode = 0;
  double distance = 0;
  double angleRobotRelative = 0;

  double angleRobot;

  double turretVectorDistance = constants.kg_TurretOffset.k_TurretOffsetPose2d.getTranslation().getDistance(new Translation2d(0,0));
  double turretVectorAngle = Math.atan2(constants.kg_TurretOffset.k_TurretOffsetPose2d.getY(), constants.kg_TurretOffset.k_TurretOffsetPose2d.getX());

  double xTurretAdjustment;
  double yTurretAdjustment;

  Pose2d turretPose;

  double angleFieldRelative;

  double[] output = {mode, distance, angleRobotRelative};

  //turret stuff
  double gearRatio = 37.6;
  double givenTarget;
  double modifier = this.gearRatio;
  double potentialTarget;

    //the interpolating double maps for scoring and passing do not touch without mentor approval
  private static final InterpolatingDoubleTreeMap scoringMapHood = new InterpolatingDoubleTreeMap();
  static {
    scoringMapHood.put(1.3, 0.0);//distance angle in rotations
    scoringMapHood.put(2.3, 1.65);
    scoringMapHood.put(2.8, 2.5);
    scoringMapHood.put(3.3, 2.7);
    scoringMapHood.put(3.8, 2.7);
    scoringMapHood.put(4.3, 4.15);
    scoringMapHood.put(5.3, 5.15);
    scoringMapHood.put(5.7,5.4);

  }
  private static final InterpolatingDoubleTreeMap passingMapHood = new InterpolatingDoubleTreeMap();
  static {
    passingMapHood.put(0.0, 7.0);//distance angle in rotations
    passingMapHood.put(100.0, 7.0);
  }

  //the interpolating double maps for scoring and passing do not touch without mentor approval
  private static final InterpolatingDoubleTreeMap scoringMapShooter = new InterpolatingDoubleTreeMap();
  static {
    scoringMapShooter.put(1.3, 17.0);
    scoringMapShooter.put(2.3, 19.0);
    scoringMapShooter.put(2.8, 20.0);
    scoringMapShooter.put(3.3, 21.5);
    scoringMapShooter.put(3.8, 23.5);
    scoringMapShooter.put(4.3, 25.5);
    scoringMapShooter.put(5.3, 29.75);
    scoringMapShooter.put(5.7, 30.0);//distance rps in rps
  }
  private static final InterpolatingDoubleTreeMap passingMapShooter = new InterpolatingDoubleTreeMap();
  static {
    passingMapShooter.put(4.0, 20.0);
    passingMapShooter.put(6.0, 21.0);//distance rps in rps
    passingMapShooter.put(7.0, 23.0);//distance rps in rps
    passingMapShooter.put(8.0, 25.0);
    passingMapShooter.put(9.0, 30.0);//distance rps in rps
    passingMapShooter.put(10.0, 35.0);//distance rps in rps
    passingMapShooter.put(12.0, 40.0);
    passingMapShooter.put(15.0, 45.0);//distance rps in rps
  }

  /** Creates a new DistanceAndAngleCalcSubsystem. */
  public DistanceAndAngleCalcSubsystem(CommandSwerveDrivetrain drivetrain, 
    HoodSubsystem hoodsubsystem, ShooterSubsystem shootersubsystem, TurretSubsystem turretsubsystem) {
    this.drivetrain = drivetrain;
    this.hoodsubsystem = hoodsubsystem;
    this.shootersubsystem = shootersubsystem;
    this.turretsubsystem = turretsubsystem;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double[] output = distanceAndAngleCalc(this.drivetrain);

    SmartDashboard.putNumber("Mode", output[0]);
    SmartDashboard.putNumber("Distance", output[1]);
    SmartDashboard.putNumber("Angle", output[2]);  

    if (output[0] == 0) {
      this.hoodsubsystem.enablemotionmagic(scoringMapHood.get(output[1]));
      this.shootersubsystem.setTargetMethod(() -> scoringMapShooter.get(output[1]));
    } else {
      this.hoodsubsystem.enablemotionmagic(passingMapHood.get(output[1]));
      this.shootersubsystem.setTargetMethod(() -> passingMapShooter.get(output[1]));
    }

    this.turretsubsystem.enablemotionmagic(calcTurretAngle(output[2]));

  }

  private double calcTurretAngle(double outputAngle){
    this.givenTarget = outputAngle * this.gearRatio / 360;
    this.modifier = this.gearRatio;
    this.potentialTarget = givenTarget;

    if (this.givenTarget >= constants.kg_TurretSubsystem.k_Motor1ReverseSoftLimitThreshold 
        && this.givenTarget <= constants.kg_TurretSubsystem.k_Motor1ForwardSoftLimitThreshold) {
      //do nothing
    } else{
      for (int i = -4; i < 5; i++) {
        this.potentialTarget =  this.givenTarget + (modifier * i);
        if (this.potentialTarget >= constants.kg_TurretSubsystem.k_Motor1ReverseSoftLimitThreshold 
            && this.potentialTarget <= constants.kg_TurretSubsystem.k_Motor1ForwardSoftLimitThreshold) {
          break;
        }
      }

    }

    return potentialTarget;
  }

  //mode then distance then angle
  private double[] distanceAndAngleCalc(CommandSwerveDrivetrain drivetrain){
    state = drivetrain.getState();
    robotPose = state.Pose;

    //figuring out where we should be targeting based on where we are on the field
    if (isAllianceRed() == 1) {
        //red alliance
        if (robotPose.getX() > constants.kg_TargetsAndField.k_RedAllianceZone) {
            mode = 0;
            targetPose = constants.kg_TargetsAndField.k_HubPose2dRed;
        } else {
            mode = 1;
            if (robotPose.getY() > constants.kg_TargetsAndField.k_MiddleOfField) {
                //in left side of field shoot to left red target
                targetPose = constants.kg_TargetsAndField.k_PassRightPose2dRed;
            } else {
                targetPose = constants.kg_TargetsAndField.k_PassLeftPose2dRed;
            }
        }
    } else {
        //blue alliance
        if (robotPose.getX() < constants.kg_TargetsAndField.k_BlueAllianceZone) {
            mode = 0;
            targetPose = constants.kg_TargetsAndField.k_HubPose2dBlue;
        } else {
            mode = 1;
            if (robotPose.getY() > constants.kg_TargetsAndField.k_MiddleOfField) {
                //in right side of field shoot to right red target
                targetPose = constants.kg_TargetsAndField.k_PassLeftPose2dBlue;
            } else {
                targetPose = constants.kg_TargetsAndField.k_PassRightPose2dBlue;
            }
        }
    }

    //all this code is to calculate where the center of the turret is in relataion to the field
    angleRobot = state.Pose.getRotation().getRadians();

    xTurretAdjustment = Math.cos(turretVectorAngle + angleRobot) * turretVectorDistance;
    yTurretAdjustment = Math.sin(turretVectorAngle + angleRobot) * turretVectorDistance;

    turretPose = new Pose2d(robotPose.getX() + xTurretAdjustment, robotPose.getY() + yTurretAdjustment, null);

    //finally calculating the distance and angle to the target
    distance = turretPose.getTranslation().getDistance(targetPose.getTranslation());

    angleFieldRelative = Math.atan2(targetPose.getY() - turretPose.getY(), targetPose.getX() - turretPose.getX());
    angleRobotRelative = Math.toDegrees((angleFieldRelative - angleRobot) * -1);//got to convert it into turret space which is the opposite if what wpi uses

    output[0] = mode;
    output[1] = distance;
    output[2] = angleRobotRelative;

    return output;
  }

  private static int isAllianceRed() {
    var alliance = DriverStation.getAlliance().orElse(Alliance.Red);
    if (alliance == DriverStation.Alliance.Red) {
        return 1;
    } else {
        return 0;
    }
  }
}
