package OpModes.AutoBlue;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

import SubSystems.AprilTagDetectionPipeline;

@Config
@Autonomous(name = "AutoBlueLeft" , group = "Blue Section")
public class AutoBlueLeft extends LinearOpMode {


    public static boolean useCamera = true;

    public static double startPoseX = -36, startPoseY = -72, startPoseAngle = 90;

    public static double startConeDeliveryPoseX = -34, startConeDeliveryPoseY = -10, startConeDeliveryAngle = 0;
    public static double startConeDeliveryPoseHelpX = -32, startConeDeliveryPoseHelpY = -10;

    public static double CONE_DELIVERY_X = -36, CONE_DELIVERY_Y = -10, CONE_DELIVERY_ANGLE = 0;

    public static double POSE_CONE_INTAKEX = -63, POSE_CONE_INTAKEY = -21, POSE_CONE_ANGLE = 180;
    public static double POSE_INTAKE_HELPX = -66, POSE_INTAKE_HELPY = -21;
    public static double poseIntakeRotationAngle = 2;

    public static double returnIntakeX = -63, returnIntakeY = -21;

    public static double poseDeliveryX = -29, poseDeliveryY = -12;

    public static double poseParkX1 = -66, poseParkY1 = -21, poseParkAngle1 = 90;
    public static double poseParkX2 = -36, poseParkY2 = -10, poseParkAngle2 = 90;
    public static double poseParkX3 = -16, poseParkY3 = -18, poseParkAngle3 = 90;

    public static double parkHelpX = -28, parkHelpY = -10;

    public static double TARGET_RESET = 0.0;
    public static double TARGET_OPEN = 0.5;
    public static double TARGET_CLOSE = 0;

    public static int minLevel = 1200, midLevel = 1990, highLevel = 2800, target = 0;
    public static int fifthCone = 455, fourthCone = 307, thirdCone = 210, secondCone = 110, firstCone = 0;
    public static double power = 1;

    public static boolean tot = true;
    DcMotor mE = null;
    Servo sG = null;

    enum ParkIn {
        LEFT,
        CENTER,
        RIGHT
    }

    ParkIn parkIn = ParkIn.CENTER;

    public static double delayBetweenActions = .5;
    public static double offset = 1;
    public static double posiionX_offset = -26, posiionY_offset = -12, posiioAngle_offset = 1;
    public static double deliveryX_offset = 3, deliveryY_offset = 0;

    public static double gripOpenDelay = 0.65;
    public static double gripCloseDelay = 0.85;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    AprilTagDetection tagOfInterest = null;


    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    final int ID_TAG_OF_INTEREST1 = 0 , ID_TAG_OF_INTEREST2 = 1, ID_TAG_OF_INTEREST3 = 2;

    @Override
    public void runOpMode() throws InterruptedException {

        if (useCamera)
        {
            initCamera();
        }


        mE = hardwareMap.get(DcMotor.class, "mE");
        sG = hardwareMap.get(Servo.class, "Grip");

        sG.setPosition(TARGET_RESET);

        mE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mE.setDirection(DcMotor.Direction.REVERSE);
        mE.setTargetPosition(target);


        SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap);


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


        driveTrain.setPoseEstimate(startPose);

        MarkerCallback ElevatorMax = new MarkerCallback() {
            @Override
            public void onMarkerReached() {
                mE.setTargetPosition(highLevel);
                mE.setPower(power);
                mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }
        };

        MarkerCallback FourthCone = new MarkerCallback() {
            @Override
            public void onMarkerReached() {
                mE.setTargetPosition(fourthCone);
                mE.setPower(power);
                mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }
        };


        MarkerCallback FifthCone = new MarkerCallback() {
            @Override
            public void onMarkerReached() {
                mE.setTargetPosition(fifthCone);
                mE.setPower(power);
                mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }
        };

        MarkerCallback ThirdCone = new MarkerCallback() {
            @Override
            public void onMarkerReached() {
                mE.setTargetPosition(thirdCone);
                mE.setPower(power);
                mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }
        };

        MarkerCallback SecondCone = new MarkerCallback() {
            @Override
            public void onMarkerReached() {
                mE.setTargetPosition(secondCone);
                mE.setPower(power);
                mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }
        };

        MarkerCallback FirstCone = new MarkerCallback() {
            @Override
            public void onMarkerReached() {
                mE.setTargetPosition(firstCone);
                mE.setPower(power);
                mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }
        };

        MarkerCallback ElevatorReset = new MarkerCallback() {
            @Override
            public void onMarkerReached() {
                mE.setTargetPosition(target);
                mE.setPower(power);
                mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            }
        };

        MarkerCallback gripOpen = new MarkerCallback() {
            @Override
            public void onMarkerReached() {
                sG.setPosition(TARGET_OPEN);

            }
        };

        MarkerCallback gripcClose = new MarkerCallback() {
            @Override
            public void onMarkerReached() {
                sG.setPosition(TARGET_CLOSE);

            }
        };

        TrajectorySequence firstConeCycle = driveTrain.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(firstCycleBarPose)
                .addTemporalMarker(ElevatorMax)
                .waitSeconds(delayBetweenActions)
                .strafeTo(firstCycleHelp)
                .waitSeconds(gripOpenDelay)
                .addTemporalMarker(gripOpen)
                .waitSeconds(delayBetweenActions)
                .splineToLinearHeading(coneIntake, poseIntakeRotationAngle)
                .strafeTo(intakeHelp)
                .addTemporalMarker(FifthCone)
                .build();


        TrajectorySequence ConeCycles = driveTrain.trajectorySequenceBuilder(firstConeCycle.end())

                //Cycle 1
                .waitSeconds(gripCloseDelay)
                .addTemporalMarker(gripcClose)
                .waitSeconds(delayBetweenActions)
                .addTemporalMarker(ElevatorMax)
                .strafeTo(intakeReturn)
                .lineToLinearHeading(ConeDelivery)
                .lineToLinearHeading(poseOffest)
                .waitSeconds(delayBetweenActions)
                .strafeTo(cycleHelp)
                .waitSeconds(gripOpenDelay)
                .addTemporalMarker(gripOpen)

                //Cycle 2

                .waitSeconds(delayBetweenActions)
                .lineToLinearHeading(ConeDelivery)
                .waitSeconds(delayBetweenActions)
                .splineToLinearHeading(coneIntake, poseIntakeRotationAngle)
                .strafeTo(intakeHelp)
                .addTemporalMarker(FourthCone)
                .waitSeconds(gripCloseDelay)
                .addTemporalMarker(gripcClose)
                .waitSeconds(delayBetweenActions)
                .addTemporalMarker(ElevatorMax)
                .waitSeconds(delayBetweenActions)
                .strafeTo(intakeReturn)
                /*
                .lineToLinearHeading(ConeDelivery)
                .lineToLinearHeading(poseOffest)
                .waitSeconds(delayBetweenActions)
                .strafeTo(cycleHelp2)
                .waitSeconds(gripOpenDelay)
                .addTemporalMarker(gripOpen)

                 */

                //Cycle 3
                /*
                .waitSeconds(delayBetweenActions)
                .lineToLinearHeading(ConeDelivery)
                .waitSeconds(delayBetweenActions )
                .splineToLinearHeading(coneIntake, poseIntakeRotationAngle)
                .strafeTo(intakeHelp)
                .addTemporalMarker(ThirdCone)
                .waitSeconds(delayBetweenActions)
                .addTemporalMarker(gripcClose)
                .waitSeconds(delayBetweenActions)
                .addTemporalMarker(ElevatorMax)
                .waitSeconds(delayBetweenActions)
                .strafeTo(intakeReturn)
                .lineToLinearHeading(ConeDelivery)
                .lineToLinearHeading(poseOffest)
                .waitSeconds(offset)
                .strafeTo(cycleHelp2)
                .waitSeconds(offset)
                .addTemporalMarker(gripOpen)
                .waitSeconds(delayBetweenActions)

                 */

                //Cycle 4

                /*
                .waitSeconds(offset)
                .lineToLinearHeading(coneIntake)
                .strafeTo(intakeHelp)
                .addTemporalMarker(SecondCone)
                .waitSeconds(offset)
                .addTemporalMarker(gripcClose)
                .lineToLinearHeading(ConeDelivery)
                .waitSeconds(delayBetweenActions)
                .addTemporalMarker(ElevatorMax)
               .waitSeconds(delayBetweenActions)
                .strafeTo(cycleHelp2)
                .waitSeconds(offset)
                .addTemporalMarker(gripOpen)

                 */
                .build();


        TrajectorySequence park1 = driveTrain.trajectorySequenceBuilder(ConeCycles.end())
                .strafeTo(parkHelp)
                .waitSeconds(delayBetweenActions)
                .addTemporalMarker(ElevatorReset)
                .lineToLinearHeading(posePark1)
                .build();

        TrajectorySequence park2 = driveTrain.trajectorySequenceBuilder(ConeCycles.end())
                .strafeTo(parkHelp)
                .waitSeconds(delayBetweenActions)
                .addTemporalMarker(ElevatorReset)
                .lineToLinearHeading(posePark2)
                .build();

        TrajectorySequence park3 = driveTrain.trajectorySequenceBuilder(ConeCycles.end())

                .strafeTo(parkHelp)
                .waitSeconds(delayBetweenActions)
                .addTemporalMarker(ElevatorReset)
                .lineToLinearHeading(posePark3)
                .build();

        if (isStopRequested()) {return;}

        if(useCamera)
        {
            lookWhereToPark();
        }

        waitForStart();

        driveTrain.followTrajectorySequence(firstConeCycle);
        driveTrain.followTrajectorySequence(ConeCycles);


        if (useCamera) {
            switch (parkIn) {
                case LEFT:
                    driveTrain.followTrajectorySequence(park1);
                    break;
                case RIGHT:
                    driveTrain.followTrajectorySequence(park3);
                    break;
                case CENTER:
                default:
                    driveTrain.followTrajectorySequence(park2);
                    break;
            }
        } else {
            driveTrain.followTrajectorySequence(park2);
        }

        while (opModeIsActive()) ;
    }

    void lookWhereToPark() {
        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    switch (tag.id) {
                        case ID_TAG_OF_INTEREST1:
                        default:
                            parkIn = ParkIn.CENTER;
                            break;
                        case ID_TAG_OF_INTEREST2:
                            parkIn = ParkIn.RIGHT;
                            break;
                        case ID_TAG_OF_INTEREST3:
                            parkIn = ParkIn.LEFT;
                            break;
                    }
                }

                switch (parkIn) {
                    case LEFT:
                        telemetry.addLine("Parking In Left Zone!");
                        break;
                    case CENTER:
                        telemetry.addLine("Parking In Center Zone(May not see tag)!");
                        break;
                    case RIGHT:
                        telemetry.addLine("Parking In Right Zone!");
                        break;
                }

            }

            telemetry.update();
            sleep(20);
        }
    }




    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    void initCamera() {
        // Lens intrinsics
        // UNITS ARE PIXELS
        // NOTE: this calibration is for the C920 webcam at 800x448.
        // You will need to do your own calibration for other configurations!
        double fx = 578.272;
        double fy = 578.272;
        double cx = 402.145;
        double cy = 221.506;

        // UNITS ARE METERS
        double tagsize = 0.166;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }
}
