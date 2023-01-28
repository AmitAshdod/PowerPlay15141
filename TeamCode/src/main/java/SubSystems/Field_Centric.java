package SubSystems;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Field_Centric")
public class Field_Centric extends LinearOpMode {

    // DcMotors
    private DcMotor mFL = null; // Front Left Dc Motor
    private DcMotor mFR = null; // Front Right Dc Motor
    private DcMotor mBL = null; // Back Left Dc Motor
    private DcMotor mBR = null; // Back Right Dc Motor
    private DcMotor mE = null; // Elevator DcMotor

    // servos
    private Servo right_grip = null; // right Servo Pinch
    private Servo left_grip = null; // left Servo Pinch

    cGamepad m1 = null;

    @Override
    public void runOpMode() throws InterruptedException {

        m1 = new cGamepad(gamepad1); // one push GamePad

        // Make sure your ID's match your configuration
        mFL = hardwareMap.get(DcMotor.class, "mFL");
        mFR = hardwareMap.get(DcMotor.class, "mFR");
        mBL = hardwareMap.get(DcMotor.class, "mBL");
        mBR = hardwareMap.get(DcMotor.class, "mBR");
        mE = hardwareMap.get(DcMotor.class, "mE");

        right_grip = hardwareMap.get(Servo.class, "right_grip");
        left_grip = hardwareMap.get(Servo.class, "left_grip");

        // DcMotors directions
        mFL.setDirection(DcMotor.Direction.REVERSE);
        mFR.setDirection(DcMotor.Direction.FORWARD);
        mBL.setDirection(DcMotor.Direction.REVERSE);
        mBR.setDirection(DcMotor.Direction.FORWARD);
        mE.setDirection(DcMotor.Direction.REVERSE);

        // servo direction
        right_grip.setDirection(Servo.Direction.REVERSE);
        left_grip.setDirection(Servo.Direction.FORWARD);


        // DcMotors Zero Power Behavior
        mFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // gyro calibration
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        waitForStart();



        if (isStopRequested()) return;
        while (opModeIsActive()) {

            m1.update();
            field_centric(imu, gamepad1);
            Elevator(gamepad2.y, gamepad2.a, 1);

        }
    }

    public int run_Mode_calc(boolean doit)
    {
        if (doit) {
            Globals.run_Mode = Globals.run_Mode + 1;
            Globals.run_Mode = Globals.run_Mode % 2;
        }

        return Globals.run_Mode;
    }

    public static class Globals
    {
        public static int run_Mode = 0;
    }

    public void field_centric_stick(@NonNull BNO055IMU imu, com.qualcomm.robotcore.hardware.Gamepad gamepad)
    {
        double tt = gamepad.right_trigger/2;
        double y = -gamepad.left_stick_y; // Remember, this is reversed!
        double x = gamepad.left_stick_x; // Counteract imperfect strafing
        double rx = gamepad.right_stick_x / 2;

        double angle = -imu.getAngularOrientation().firstAngle;

        double rotX = x * Math.cos(angle) - y * Math.sin(angle);
        rotX = (rotX * tt) + (rotX / 2);

        double rotY = x * Math.sin(angle) + y * Math.cos(angle);
        rotY = (rotY * tt) + (rotY / 2);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        mFL.setPower((rotY + rotX + rx)/ denominator);
        mBL.setPower((rotY - rotX + rx)/ denominator);
        mFR.setPower((rotY - rotX - rx)/ denominator);
        mBR.setPower((rotY + rotX - rx)/ denominator);

        telemetry.addData("global heading: ", angle);
        telemetry.addData("x ", x);
        telemetry.addData("y ", y);
    }

    public void field_centric_dpad(@NonNull BNO055IMU imu, com.qualcomm.robotcore.hardware.Gamepad gamepad)
    {
        double rx = gamepad.right_stick_x;
        double y = 0;// Remember, this is reversed!
        double x = 0;

        if (gamepad.dpad_down && !gamepad.dpad_up) y = -1;
        else if (gamepad.dpad_up && !gamepad.dpad_down) y = 1;
        else y = 0;

        if (gamepad.dpad_left && !gamepad.dpad_right) x = -1;
        else if (gamepad.dpad_right && !gamepad.dpad_left) x = 1;
        else x = 0;


        double angle = -imu.getAngularOrientation().firstAngle;

        double rotX = x * Math.cos(angle) - y * Math.sin(angle);
        double rotY = x * Math.sin(angle) + y * Math.cos(angle);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        mFL.setPower((rotY + rotX + rx)/ denominator);
        mBL.setPower((rotY - rotX + rx)/ denominator);
        mFR.setPower((rotY - rotX - rx)/ denominator);
        mBR.setPower((rotY + rotX - rx)/ denominator);

        telemetry.addData("global heading: ", angle);
        telemetry.addData("x ", x);
        telemetry.addData("y ", y);
    }

    public void field_centric(@NonNull BNO055IMU imu, com.qualcomm.robotcore.hardware.Gamepad gamepad)
    {
        if (gamepad.dpad_right || gamepad.dpad_left || gamepad.dpad_up || gamepad.dpad_down) field_centric_dpad(imu, gamepad);
        else field_centric_stick(imu, gamepad);
    }

    public void Elevator(boolean upMove, boolean downMove, double power)
    {
        if (upMove && !downMove) //moving up
        {
            mE.setPower(power);
        }
        else if (downMove && !upMove) //moving down
        {
            mE.setPower(-power);
        }
        else //default state
        {
            mE.setPower(0);
        }
    }

    public void grip()
    {
        telemetry.addData("right: ", right_grip.getPosition());
        telemetry.addData("left: ", left_grip.getPosition());

        if (gamepad2.right_bumper)
        {
            right_grip.setPosition(0.5);
            left_grip.setPosition(0.5);
        }
        else if(gamepad2.left_bumper)
        {
            right_grip.setPosition(0);
            left_grip.setPosition(0.0);
        }
    }
}