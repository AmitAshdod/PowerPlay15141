package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepREDLEFT {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(550);

        double startPoseX = -36, startPoseY = -72, startPoseAngle = 90;
        double startConeDeliveryPoseX = -36, startConeDeliveryPoseY = 0, startConeDeliveryAngle = 0;
        double coneDeliveryPoseX = -32, coneDeliveryPoseY = -12, coneDeliveryAngle = 45;
        double coneIntakePoseX = -60, coneIntakePoseY = -16, coneIntakeAngle = 0;

        Pose2d startPose = new Pose2d(startPoseX,startPoseY, Math.toRadians(startPoseAngle));

        Pose2d firstCycleBarPose = new Pose2d(startConeDeliveryPoseX,startConeDeliveryPoseY, Math.toRadians(startConeDeliveryAngle));
        Pose2d secondCycleBarPose = new Pose2d(coneDeliveryPoseX,coneDeliveryPoseY,Math.toRadians(coneDeliveryAngle));
        Pose2d coneStackPose = new Pose2d(coneIntakePoseX,coneIntakePoseY, Math.toRadians(coneIntakeAngle));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)

                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                //First Cycle
                                .lineToLinearHeading(firstCycleBarPose)
                                //Second cycle
                                .splineToLinearHeading(coneStackPose,Math.toRadians(startPoseAngle))
                                .lineToLinearHeading(secondCycleBarPose)
                                //Third cycle
                                .lineToLinearHeading(coneStackPose)
                                .lineToLinearHeading(secondCycleBarPose)

                                .build()
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}