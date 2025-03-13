// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

public class FieldUtils{
    private static FieldUtils m_fieldUtils;

    public static final AllianceAprilTags RedTags =
        new AllianceAprilTags(
            1,
            2,
            3,
            4,
            5,
            6,
            7,
            8,
            11,
            10,
            9);
    public static final AllianceAprilTags BlueTags = 
        new AllianceAprilTags(
            12,
            13,
            14,
            15,
            16,
            19,
            18,
            17,
            20, 
            21, 
            22);

    public static FieldUtils getInstance(){
        if(m_fieldUtils == null){
            m_fieldUtils = new FieldUtils();
        }
        return m_fieldUtils;
    }

    private FieldUtils(){}

    public AllianceAprilTags getAllianceAprilTags(){
        AllianceAprilTags tags = BlueTags;
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if(alliance.isPresent()){
            if(alliance.get() == DriverStation.Alliance.Red){
                tags = RedTags;
            } else if(alliance.get() == DriverStation.Alliance.Blue) {
                tags = BlueTags;
            }
        }
        return tags;
    }

    public Pose3d getTagPose(int tagId)
    {
        return Constants.VisionConstants.kTagLayout.getTagPose(tagId).get();
    }
    
    public Rotation2d getRotationOffset() {
        Rotation2d offset = new Rotation2d();//returns no offset by default
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if(alliance.isPresent() && 
            alliance.get() == DriverStation.Alliance.Red){
                offset = new Rotation2d(Math.PI);
        }
        return offset;
    }

    public Rotation2d getAngleToPose(Pose2d currentPose, Pose2d targetPose) {
        Translation2d curTrans = currentPose.getTranslation();
        Translation2d targetTrans = targetPose.getTranslation();
        Translation2d toTarget = targetTrans.minus(curTrans);
        return toTarget.getAngle();
    }

    public enum ReefFaceOffset {
        kLeft,
        kRight
    }

    /**
     * Returns the best scoring postion on the reef based on current robot pose
     * @param currentRobotPose
     * @param offsetDirection
     * @return the Pose2d to score at, or null if scoring position couldn't be found
     */
    public Pose2d getClosestReefScoringPosition(Pose2d currentRobotPose, ReefFaceOffset offsetDirection, Alliance alliance) {
        if(!inAllianceHalf(currentRobotPose, alliance)) {
            return null;
        }
        int sextant = getSextant(currentRobotPose, alliance);

        Pose2d faceTagPose = new Pose2d();
        switch (alliance) {
            case Blue:
                faceTagPose = getBlueFaceTag(sextant);
                break;
            case Red:
                faceTagPose = getRedFaceTag(sextant);
                break;
        }

        Translation2d scoreTranslation = ((offsetDirection==ReefFaceOffset.kLeft)? 
                                            Constants.FieldLocationConstants.kReefLeftScoreTrans :
                                            Constants.FieldLocationConstants.kReefRightScoreTrans);
        Transform2d scoreTransform = new Transform2d(scoreTranslation, Rotation2d.k180deg);
        Pose2d scoringPose = faceTagPose.transformBy(scoreTransform);

        return scoringPose;
    }

    private Pose2d getBlueFaceTag(int sextant)
    {
        switch (sextant) {
            case 0:
                return getTagPose(getAllianceAprilTags().middleFrontReef).toPose2d();
            case 1:
                return getTagPose(getAllianceAprilTags().rightFrontReef).toPose2d();
            case 2:
                return getTagPose(getAllianceAprilTags().rightBackReef).toPose2d();
            case 3:
                return getTagPose(getAllianceAprilTags().middleBackReef).toPose2d();
            case 4:
                return getTagPose(getAllianceAprilTags().leftBackReef).toPose2d();
            case 5:
                return getTagPose(getAllianceAprilTags().leftFrontReef).toPose2d();
            default:
                return null;
        }
    }
    private Pose2d getRedFaceTag(int sextant)
    {
        switch (sextant) {
            case 0:
                return getTagPose(getAllianceAprilTags().middleBackReef).toPose2d();
            case 1:
                return getTagPose(getAllianceAprilTags().leftBackReef).toPose2d();
            case 2:
                return getTagPose(getAllianceAprilTags().leftFrontReef).toPose2d();
            case 3:
                return getTagPose(getAllianceAprilTags().middleFrontReef).toPose2d();
            case 4:
                return getTagPose(getAllianceAprilTags().rightFrontReef).toPose2d();
            case 5:
                return getTagPose(getAllianceAprilTags().rightBackReef).toPose2d();
            default:
                return null;
        }
    }

    public int getSextant(Pose2d robotPose, Alliance alliance) {
        Pose2d reefCenter = (alliance == Alliance.Blue) ? 
        Constants.FieldLocationConstants.kBlueReefCenter :
        Constants.FieldLocationConstants.kRedReefCenter;

        Rotation2d angle = getAngleToPose(reefCenter, robotPose);
        Rotation2d offsetAngle = angle.plus(new Rotation2d(Units.degreesToRadians(30)));

        return ((int)Math.floor((offsetAngle.getDegrees() / 360) * 6)) + 3;
    }

    public boolean inAllianceHalf(Pose2d robotPose, Alliance alliance) {
        return (alliance == Alliance.Blue) ^ (robotPose.getX() > Constants.FieldLocationConstants.kMidfieldX);
    }

    public TargetPose getRedCoralA1Pose() {
        return Constants.FieldLocationConstants.kRedCoralA1Pose;
    }

    public TargetPose getRedCoralA2Pose() {
        return Constants.FieldLocationConstants.kRedCoralA2Pose;
    }
}