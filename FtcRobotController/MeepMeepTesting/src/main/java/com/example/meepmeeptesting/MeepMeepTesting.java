package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    //start values.
    public static double startPoseX = -36, startPoseY = -72, startPoseAngle = 90;

    /* NOT FINAL*/
    public static final double coneDeliveryPoseX = -36, coneDeliveryPoseY = 0, coneDeliveryAngle = 0;
    public static double coneIntakePoseX = -56, coneIntakePoseY = -12;
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(550);
        Pose2d startPose = new Pose2d(startPoseX,startPoseY, Math.toRadians(startPoseAngle));

        Pose2d firstCycleBarPose = new Pose2d(coneIntakePoseX,coneIntakePoseY, Math.toRadians(startPoseAngle));
        Vector2d secondCycleBarVector = new Vector2d(coneDeliveryPoseX,coneDeliveryPoseY);
        Vector2d coneStackVector = new Vector2d(coneIntakePoseX,coneIntakePoseY);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .splineToLinearHeading(firstCycleBarPose,coneDeliveryAngle)
                                .lineTo(coneStackVector)
                                .lineTo(secondCycleBarVector)

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}