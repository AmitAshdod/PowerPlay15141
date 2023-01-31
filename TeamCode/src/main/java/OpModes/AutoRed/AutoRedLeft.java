package OpModes.AutoRed;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "AutoRedLeft")
public class AutoRedLeft extends LinearOpMode {

    public static double startPoseX = -36, startPoseY = -72, startPoseAngle = 90;

    public static double startConeDeliveryPoseX = -34, startConeDeliveryPoseY = -10, startConeDeliveryAngle = 0;
    public static double startConeDeliveryPoseHelpX = -30, startConeDeliveryPoseHelpY = -10;

    public static double CONE_DELIVERY_X = -36, CONE_DELIVERY_Y = -10, CONE_DELIVERY_ANGLE = 0;

    public static double POSE_CONE_INTAKEX = -63, POSE_CONE_INTAKEY = -21, POSE_CONE_ANGLE = 180;
    public static double POSE_INTAKE_HELPX = -66, POSE_INTAKE_HELPY = -21;

    public static double poseParkX = -34, poseParkY = -10, poseParkAngle = 90;

    public static double poseDeliveryX = -34, poseDeliveryY = -10 ;


    public static double TARGET_RESET = 0.0;
    public static double TARGET_OPEN = 0.5;
    public static double TARGET_CLOSE = 0;

    public static int minLevel = 1200, midLevel = 1990, highLevel = 2800, target = 0;
    public static int fifthCone = 455 , fourthCone = 307, thirdCone = 210, secondCone = 110 , firstCone = 0;
    public static double power = 1;

    DcMotor mE = null;
    Servo sG = null;

    public static double  delayBetweenActions = 1;
    public static double  offset = 1;
    public static double  posiionX_offset = 1;
    public static double  posiionY_offset = 1;


    @Override
    public void runOpMode() throws InterruptedException {

        mE = hardwareMap.get(DcMotor.class, "mE");
        sG = hardwareMap.get(Servo.class, "Grip");

        sG.setPosition(TARGET_RESET);

        mE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mE.setDirection(DcMotor.Direction.REVERSE);
        mE.setTargetPosition(target);


        SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap);


        Pose2d startPose = new Pose2d(startPoseX,startPoseY, Math.toRadians(startPoseAngle));
        Pose2d firstCycleBarPose = new Pose2d(startConeDeliveryPoseX,startConeDeliveryPoseY, Math.toRadians(startConeDeliveryAngle));
        Pose2d coneIntake = new Pose2d(POSE_CONE_INTAKEX, POSE_CONE_INTAKEY, Math.toRadians(POSE_CONE_ANGLE));
        Pose2d ConeDelivery = new Pose2d(CONE_DELIVERY_X, CONE_DELIVERY_Y, Math.toRadians(CONE_DELIVERY_ANGLE));
        Pose2d posePark = new Pose2d(poseParkX, poseParkY, Math.toRadians(poseParkAngle));

        Vector2d cycleHelp = new Vector2d(poseDeliveryX, poseDeliveryY);
        Vector2d firstCycleHelp = new Vector2d( startConeDeliveryPoseHelpX ,startConeDeliveryPoseHelpY);
        Vector2d intakeHelp = new Vector2d( POSE_INTAKE_HELPX, POSE_INTAKE_HELPY);
        Vector2d cycleHelp2 = new Vector2d(poseDeliveryX + 1, poseDeliveryY + 1);

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
                .waitSeconds(offset)
                .strafeTo(firstCycleHelp)
                .waitSeconds(offset)
                .addTemporalMarker(gripOpen)
                .waitSeconds(delayBetweenActions )
                .lineToLinearHeading(coneIntake)
                .strafeTo(intakeHelp)
                .addTemporalMarker(FifthCone)
                .waitSeconds(offset)
                .build();



       TrajectorySequence ConeCycles = driveTrain.trajectorySequenceBuilder(firstConeCycle.end())

               //Cycle 1

                .addTemporalMarker(gripcClose)
                .waitSeconds(delayBetweenActions)
                .addTemporalMarker(ElevatorMax)
                .lineToLinearHeading(ConeDelivery)
                .waitSeconds(offset)
                .strafeTo(cycleHelp)
                .waitSeconds(offset)
                .addTemporalMarker(gripOpen)

               //Cycle 2

                .waitSeconds(delayBetweenActions)
                .lineToLinearHeading(coneIntake)
                .strafeTo(intakeHelp)
                .addTemporalMarker(FourthCone)
                .waitSeconds(delayBetweenActions)
                .addTemporalMarker(gripcClose)
                .waitSeconds(delayBetweenActions)
                .addTemporalMarker(ElevatorMax)
                .waitSeconds(delayBetweenActions)
                .lineToLinearHeading(ConeDelivery)
                .waitSeconds(delayBetweenActions)
                .strafeTo(cycleHelp2)
                .waitSeconds(delayBetweenActions)
                .addTemporalMarker(gripOpen)

               //Cycle 3

                .waitSeconds(delayBetweenActions)
                .lineToLinearHeading(coneIntake)
                .strafeTo(intakeHelp)
                .addTemporalMarker(ThirdCone)
                .waitSeconds(delayBetweenActions)
               .addTemporalMarker(gripcClose)
               .waitSeconds(delayBetweenActions)
                .addTemporalMarker(ElevatorMax)
                .waitSeconds(offset)
                .addTemporalMarker(gripcClose)
                .waitSeconds(delayBetweenActions)
                .lineToLinearHeading(ConeDelivery)
                .waitSeconds(offset)
                .strafeTo(cycleHelp2)
                .waitSeconds(offset)
                .addTemporalMarker(gripOpen)

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

               // Parking robot to an angle of 90 degrees
                .lineToLinearHeading(posePark)
                .build();





        if(isStopRequested()){ return;}
        waitForStart();

        driveTrain.followTrajectorySequence(firstConeCycle);
        driveTrain.followTrajectorySequence(ConeCycles);




    }

}
