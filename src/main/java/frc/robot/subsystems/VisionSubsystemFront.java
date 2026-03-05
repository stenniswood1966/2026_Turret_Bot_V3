// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Field;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.LimelightHelpers;
import frc.robot.constants;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.constantsVision;

public class VisionSubsystemFront extends SubsystemBase {
  public LimelightHelpers.PoseEstimate megaTag2 = new PoseEstimate();
  public Supplier<Pigeon2> drivetrainState;
  public Consumer<Pose2d> visionPose = (message) -> doesNothing(message);
  public CommandSwerveDrivetrain drivetrain;
  
  public double testTimestamp;

  public Trigger addLimelightPose = new Trigger(this::isPoseValid);

  private double latestRobotRotationRate;
  private double latestRobotHeading;

  public static final AprilTagFieldLayout FIELD_LAYOUT;

  static {
    FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
    FIELD_LAYOUT.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
  }

  public void doesNothing(Pose2d visionPose){

  }

  /** Creates a new Vision. */
  public VisionSubsystemFront(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    
    this.setLimelightRobotPosition();
    LimelightHelpers.SetIMUMode(constantsVision.kg_Limelight1.k_LimelightName, 1);//was 1
    LimelightHelpers.SetRobotOrientation(constantsVision.kg_Limelight1.k_LimelightName, constantsVision.kg_Limelight1.k_LimelightYawOffset, 0, 0, 0, 0, 0);
  }

  @Override
  
  public void periodic() {
    // This method will be called once per scheduler run
    latestRobotHeading = this.getRobotHeading();
    latestRobotRotationRate = this.getRobotRotationRate();
    
    LimelightHelpers.SetRobotOrientation(constantsVision.kg_Limelight1.k_LimelightName, latestRobotHeading, 0.0, 0.0,0.0,0.0,0.0);
    //THIS LINE IS EXTREMELY IMPORTANT 
    //only update the megaTag if the estimate is not null
    var megaTag2Temp = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(constantsVision.kg_Limelight1.k_LimelightName);
    if (this.isMegaTag2Valid(megaTag2Temp)) {
      megaTag2 = megaTag2Temp;
    }
  }

  private void setLimelightRobotPosition(){
    LimelightHelpers.setCameraPose_RobotSpace(
      constantsVision.kg_Limelight1.k_LimelightName,
      constantsVision.kg_Limelight1.k_LimelightFowardOffset,
      constantsVision.kg_Limelight1.k_LimelightSideOffset,
      constantsVision.kg_Limelight1.k_LimelightUpOffset,
      constantsVision.kg_Limelight1.k_LimelightRollOffset,
      constantsVision.kg_Limelight1.k_LimelightPitchOffset,
      constantsVision.kg_Limelight1.k_LimelightYawOffset);
  }
  
  private double getRobotHeading(){
    return this.drivetrain.getState().Pose.getRotation().getDegrees();
  }
  
  private boolean isMegaTag2Valid(LimelightHelpers.PoseEstimate poseEstimate){
    return (poseEstimate != null) && LimelightHelpers.getTV(constantsVision.kg_Limelight1.k_LimelightName) 
      && this.isMegaTag2OutputInField(poseEstimate) && this.isTagInRange(poseEstimate) 
      && this.isBotLevel();
  }

  //this method means that the tag number is in a reasonable range aka between 0 and 22 for the 2025 field
  private boolean isTagInRange(LimelightHelpers.PoseEstimate poseEstimate){
    int tagNumber = (int) NetworkTableInstance.getDefault().getTable(constantsVision.kg_Limelight1.k_LimelightName).getEntry("tid").getDouble(-1);
    return (tagNumber >= 0) && (tagNumber <= FIELD_LAYOUT.getTags().size());//0-22 for 2025 field
  }

  private boolean isMegaTag2OutputInField(LimelightHelpers.PoseEstimate poseEstimate){
    return (poseEstimate.pose.getX() >= 0) && (poseEstimate.pose.getX() <= FIELD_LAYOUT.getFieldLength()) 
      && (poseEstimate.pose.getY() >= 0) && (poseEstimate.pose.getY() <= FIELD_LAYOUT.getFieldWidth());
  }

  public Command addMegaTag2(Supplier<CommandSwerveDrivetrain> drivetrain){
    return this.updateLimelightTelemetry().andThen(run(
      () -> {
        testTimestamp = Utils.getCurrentTimeSeconds();
        drivetrain.get().setVisionMeasurementStdDevs(VecBuilder.fill(0.5,0.5, 9999999));
        drivetrain.get().addVisionMeasurement(megaTag2.pose, megaTag2.timestampSeconds);
      }
    ).withName("Adding Vision Measurement").ignoringDisable(true));
  }

  public boolean isPoseValid() {
    return this.isMegaTag2Valid(this.megaTag2) && this.areTagsSeen(1) && this.isRobotSlowEnough(3.3);
  }

  public boolean areTagsSeen(int tagCount){
    return this.megaTag2.tagCount >= tagCount;
  }

  public boolean isRobotSlowEnough(double maximumRotationRate){
    return this.latestRobotRotationRate <= maximumRotationRate;
  }
  
  public Command updateLimelightTelemetry(){
    return runOnce(() -> {
      this.visionPose.accept(this.getPose().get());
    }).ignoringDisable(true)
    .withName("updateLimelightTelemetry");
  }
  
  private Supplier<Pose2d> getPose(){
    return this::getCurrentPose;
  }

  private Pose2d getCurrentPose(){
    return this.megaTag2.pose;
  }

  private double getRobotRotationRate(){
    return Math.abs(this.drivetrain.getState().Speeds.omegaRadiansPerSecond);
  }

  private Boolean isBotLevel(){
    if (this.drivetrain.getPigeon2().getRoll().getValueAsDouble() < constantsVision.k_PitchRollThreshold 
        && this.drivetrain.getPigeon2().getPitch().getValueAsDouble() < constantsVision.k_PitchRollThreshold) {
      return true;
    } else {
      //System.out.println("Vision Pitch Roll Cutoff");
      return false;
    }
  }

  private int isAllianceRed() {
    var alliance = DriverStation.getAlliance();
    if (alliance.get() == DriverStation.Alliance.Red) {
      return 1;
    } else {
      return 0;
    }
  }
}

