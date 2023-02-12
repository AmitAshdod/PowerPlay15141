package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public  class MeepMeepTesting {

    public static boolean useCamera = true;

    public static double startPoseX = -36, startPoseY = -72, startPoseAngle = 90;

    public static double startConeDeliveryPoseX = -34, startConeDeliveryPoseY = -10, startConeDeliveryAngle = 0;
    public static double startConeDeliveryPoseHelpX = -32, startConeDeliveryPoseHelpY = -10;

    public static double CONE_DELIVERY_X = -36, CONE_DELIVERY_Y = -10, CONE_DELIVERY_ANGLE = 0;

    public static double POSE_CONE_INTAKEX = -63, POSE_CONE_INTAKEY = -21, POSE_CONE_ANGLE = 180;
    public static double POSE_INTAKE_HELPX = -66, POSE_INTAKE_HELPY = -21;
    public static double poseIntakeRotationAngle = 2;

    public static double returnIntakeX = -63, returnIntakeY = -21;

    public static double poseDeliveryX = -30, poseDeliveryY = -12;

    public static double poseParkX1 = -15, poseParkY1 = -10 -15, poseParkAngle1 = 90;
    public static double poseParkX2 = -36, poseParkY2 = -10, poseParkAngle2 = 90;
    public static double poseParkX3 = -36, poseParkY3 = -10, poseParkAngle3 = 90;

    public static double parkHelpX = -28, parkHelpY = -10;

    public static double TARGET_RESET = 0.0;
    public static double TARGET_OPEN = 0.5;
    public static double TARGET_CLOSE = 0;

    public static int minLevel = 1200, midLevel = 1990, highLevel = 2800, target = 0;
    public static int fifthCone = 455, fourthCone = 307, thirdCone = 210, secondCone = 110, firstCone = 0;
    public static double power = 1;

    public static double delayBetweenActions = .5;
    public static double offset = 1;
    public static double posiionX_offset = -26, posiionY_offset = -12, posiioAngle_offset = 1;
    public static double deliveryX_offset = 3, deliveryY_offset = 0;

    public static double gripOpenDelay = 0.35;
    public static double gripCloseDelay = 0.85;


    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(550);

        Pose2d startPose = new Pose2d(startPoseX, startPoseY, Math.toRadians(startPoseAngle));
        Pose2d firstCycleBarPose = new Pose2d(startConeDeliveryPoseX, startConeDeliveryPoseY, Math.toRadians(startConeDeliveryAngle));
        Pose2d coneIntake = new Pose2d(POSE_CONE_INTAKEX, POSE_CONE_INTAKEY, Math.toRadians(POSE_CONE_ANGLE));
        Pose2d ConeDelivery = new Pose2d(CONE_DELIVERY_X, CONE_DELIVERY_Y, Math.toRadians(CONE_DELIVERY_ANGLE));

        Pose2d posePark1 = new Pose2d(poseParkX1, poseParkY1, Math.toRadians(poseParkAngle3));
        Pose2d posePark2 = new Pose2d(poseParkX2, poseParkY2, Math.toRadians(poseParkAngle2));
        Pose2d posePark3 = new Pose2d(poseParkX3, poseParkY3, Math.toRadians(poseParkAngle3));

        Vector2d parkHelp = new Vector2d(parkHelpX, parkHelpY);

        Pose2d poseOffest = new Pose2d(posiionX_offset, posiionY_offset, Math.toRadians(posiioAngle_offset));

        Vector2d firstCycleHelp = new Vector2d(startConeDeliveryPoseHelpX, startConeDeliveryPoseHelpY);

        Vector2d intakeHelp = new Vector2d(POSE_INTAKE_HELPX, POSE_INTAKE_HELPY);
        Vector2d intakeReturn = new Vector2d(returnIntakeX, returnIntakeY);

        Vector2d cycleHelp2 = new Vector2d(poseDeliveryX + deliveryX_offset, poseDeliveryY + deliveryY_offset);
        Vector2d cycleHelp = new Vector2d(poseDeliveryX, poseDeliveryY);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)

        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
        .setConstraints(60, 60, Math.toRadians(130), Math.toRadians(130), 13.23)
        .followTrajectorySequence(drive ->
        drive.trajectorySequenceBuilder(startPose)

        //First Cycle
        .lineToLinearHeading(firstCycleBarPose)//delivery

                //Cycle 2
                .waitSeconds(delayBetweenActions)
                .lineToLinearHeading(ConeDelivery)
                .waitSeconds(delayBetweenActions)
                .splineToLinearHeading(coneIntake, poseIntakeRotationAngle)
                .strafeTo(intakeHelp)
                .waitSeconds(gripCloseDelay)
                .strafeTo(intakeReturn)
                .lineToLinearHeading(ConeDelivery)
                .lineToLinearHeading(poseOffest)
                .waitSeconds(delayBetweenActions)
                .strafeTo(cycleHelp2)
                .waitSeconds(gripOpenDelay)


                .strafeTo(parkHelp)
                .waitSeconds(delayBetweenActions)
                .lineToLinearHeading(posePark2)
        .build()
        );


        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
        .setDarkMode(true)
        .setBackgroundAlpha(0.95f)
        .addEntity(myBot)
        .start();
        }
        }