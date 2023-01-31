package OpModes.AutoBlue;

import static SubSystems.Imu.imu;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import SubSystems.Imu;

@Config
@Autonomous(name = "AutoBlueRight")
public class AutoBlueRight extends LinearOpMode {

    public static double startPoseX = -36, startPoseY = 72, startPoseAngle = -90;
    public static double startConeDeliveryPoseX = -34, startConeDeliveryPoseY = 10, startConeDeliveryAngle = 0;
    public static double startConeDeliveryPoseHelpX = -35, startConeDeliveryPoseHelpY = 10;
    public static double parkPoseX = -40 , parkPoseY = 34, poseParkAngle = 90;

    public static double TARGET_RESET = 0.0;
    public static double TARGET_OPEN = 0.5;
    public static double TARGET_CLOSE = 0;

    int minLevel = 1200, midLevel = 1990, highLevel = 2800, target = 0;
    double power = 1;

    DcMotor mE = null;
    Servo sG = null;

    public static double DELIVERY_WAIT_TIME = 2, INTAKE_WAIT_TIME = 4, delayBetweenActions = 3;

    Imu imu = new Imu(hardwareMap, telemetry);
    double angle = Imu.imu.getAngularOrientation().firstAngle;

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
        Vector2d firstCycleHelp = new Vector2d( startConeDeliveryPoseHelpX + 5,startConeDeliveryPoseHelpY);
        Pose2d parkPose =  new Pose2d(parkPoseX, parkPoseY, Math.toRadians(poseParkAngle));


        driveTrain.setPoseEstimate(startPose);

        MarkerCallback ElevatorMax = new MarkerCallback() {
            @Override
            public void onMarkerReached() {
                mE.setTargetPosition(highLevel);
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
                .waitSeconds(delayBetweenActions - 2)
                .strafeTo(firstCycleHelp)
                .waitSeconds(delayBetweenActions - 2.5)
                .addTemporalMarker(gripOpen)
                .waitSeconds(delayBetweenActions - 2)
                .lineToLinearHeading(firstCycleBarPose)
                .addTemporalMarker(ElevatorReset)
                .lineToLinearHeading(parkPose)
                .build();




        if(isStopRequested()){ return;}
        waitForStart();

        driveTrain.followTrajectorySequence(firstConeCycle);


    }

}