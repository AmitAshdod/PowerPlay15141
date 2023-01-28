package OpModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class xyz_pose extends LinearOpMode {
    //private final SensorManager mSensorManager;
    //private final Sensor mAccelerometer;

    static final float NS2S = 1.0f / 1000000000.0f;
    float[] last_values = null;
    float[] velocity = null;
    float[] position = null;
    long last_timestamp = 0;

    private DcMotor mFL = null;
    private DcMotor mFR = null;
    private DcMotor mBL = null;
    private DcMotor mBR = null;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        mFL = hardwareMap.get(DcMotor.class, "mFL");
        mFR = hardwareMap.get(DcMotor.class, "mFR");
        mBL = hardwareMap.get(DcMotor.class, "mBL");
        mBR = hardwareMap.get(DcMotor.class, "mBR");

        mFL.setDirection(DcMotor.Direction.REVERSE);
        mFR.setDirection(DcMotor.Direction.FORWARD);
        mBL.setDirection(DcMotor.Direction.REVERSE);
        mBR.setDirection(DcMotor.Direction.FORWARD);

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double angle = -imu.getAngularOrientation().firstAngle;

            double rotX = x * Math.cos(angle) - y * Math.sin(angle);
            double rotY = x * Math.sin(angle) + y * Math.cos(angle);

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            mFL.setPower((rotY + rotX + rx)/ denominator);
            mBL.setPower((rotY - rotX + rx)/ denominator);
            mFR.setPower((rotY - rotX - rx)/ denominator);
            mBR.setPower((rotY + rotX - rx)/ denominator);



        }
    }
/*
    public void SensorActivity() {
        //mSensorManager = (SensorManager)getSystemService(SENSOR_SERVICE);
        //mAccelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
    }

    public void onSensorChanged(SensorEvent event){
        if (last_values != null) {
            float dt = (event.timestamp - last_timestamp) * NS2S;

            for (int index = 0; index < 3; ++index) {
                velocity[index] += (event.values[index] + last_values[index]) / 2 * dt;
                position[index] += velocity[index] * dt;
            }
        } else {
            last_values = new float[3];
            velocity = new float[3];
            position = new float[3];
            velocity[0] = velocity[1] = velocity[2] = 0f;
            position[0] = position[1] = position[2] = 0f;
        }
        System.arraycopy(event.values, 0, last_values, 0, 3);
        last_timestamp = event.timestamp;

        telemetry.addData("position[0] - x", position[0]);
        telemetry.addData("position[1] - y", position[1]);
        telemetry.addData("position[2] - z", position[1]);
        telemetry.update();
    }*/
}