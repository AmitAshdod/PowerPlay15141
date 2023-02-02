

        package OpModes;

        import androidx.annotation.NonNull;

        import com.acmerobotics.dashboard.config.Config;
        import com.qualcomm.hardware.bosch.BNO055IMU;
        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.Servo;

        import SubSystems.Field_Centric;
        import SubSystems.cGamepad;


@TeleOp(name="Teleop_v1")
@Config
public class Final_TeleOp extends OpMode {
    // DcMotors
    private DcMotor mFL = null; // Front Left Dc Motor
    private DcMotor mFR = null; // Front Right Dc Motor
    private DcMotor mBL = null; // Back Left Dc Motor
    private DcMotor mBR = null; // Back Right Dc Motor
    private DcMotor mE = null; // Elevator DcMotor

    // servos
    private Servo sG = null; // Servo Pinch

    private BNO055IMU imu = null;
    cGamepad m1 = null;

    public static double TARGET_RESET = 0.0;
    public static double TARGET_OPEN = 0.5;
    public static double TARGET_CLOSE = 0;

    public static int dEu = 166;
    public static double dEd = 0.2;

    @Override
    public void init() {
        m1 = new cGamepad(gamepad1); // one push GamePad

        // Make sure your ID's match your configuration
        mFL = hardwareMap.get(DcMotor.class, "mFL");
        mFR = hardwareMap.get(DcMotor.class, "mFR");
        mBL = hardwareMap.get(DcMotor.class, "mBL");
        mBR = hardwareMap.get(DcMotor.class, "mBR");
        mE = hardwareMap.get(DcMotor.class, "mE");

        sG = hardwareMap.get(Servo.class, "Grip");

        // DcMotors directions
        mFL.setDirection(DcMotor.Direction.REVERSE);
        mFR.setDirection(DcMotor.Direction.FORWARD);
        mBL.setDirection(DcMotor.Direction.REVERSE);
        mBR.setDirection(DcMotor.Direction.FORWARD);
        mE.setDirection(DcMotor.Direction.REVERSE);

        mE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // servo direction
        sG.setDirection(Servo.Direction.FORWARD);

        // DcMotors Zero Power Behavior
        mFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // gyro calibration
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }

    @Override
    public void init_loop() {
        gripReset();
        resetElevator();
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {

        m1.update();
        field_centric(imu, gamepad1);
        Elevator(gamepad2.y, gamepad2.a, 1);
        grip();
        telemetry.update();

    }

    @Override
    public void stop() {
    }

    public int run_Mode_calc(boolean doit) {
        if (doit) {
            Field_Centric.Globals.run_Mode = Field_Centric.Globals.run_Mode + 1;
            Field_Centric.Globals.run_Mode = Field_Centric.Globals.run_Mode % 2;
        }

        return Field_Centric.Globals.run_Mode;
    }

    public static class Globals {
        public static int run_Mode = 0;
        public static int temp = 0;
    }

    public void field_centric_stick(@NonNull BNO055IMU imu, com.qualcomm.robotcore.hardware.Gamepad gamepad) {
        double tt = gamepad.right_trigger / 2;
        double y = -gamepad.left_stick_y; // Remember, this is reversed!
        double x = gamepad.left_stick_x; // Counteract imperfect strafing
        double rx = gamepad.right_stick_x;
        rx = (rx * tt) + (rx / 2);

        double angle = -imu.getAngularOrientation().firstAngle;

        double rotX = x * Math.cos(angle) - y * Math.sin(angle);
        rotX = (rotX * tt) + (rotX / 2);

        double rotY = x * Math.sin(angle) + y * Math.cos(angle);
        rotY = (rotY * tt) + (rotY / 2);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        mFL.setPower((rotY + rotX + rx) / denominator);
        mBL.setPower((rotY - rotX + rx) / denominator);
        mFR.setPower((rotY - rotX - rx) / denominator);
        mBR.setPower((rotY + rotX - rx) / denominator);

        telemetry.addData("global heading: ", angle);
        telemetry.addData("x ", x);
        telemetry.addData("y ", y);
    }

    public void field_centric_dpad(@NonNull BNO055IMU imu, com.qualcomm.robotcore.hardware.Gamepad gamepad) {
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

        mFL.setPower((rotY + rotX + rx) / denominator);
        mBL.setPower((rotY - rotX + rx) / denominator);
        mFR.setPower((rotY - rotX - rx) / denominator);
        mBR.setPower((rotY + rotX - rx) / denominator);

        telemetry.addData("global heading: ", angle);
        telemetry.addData("x ", x);
        telemetry.addData("y ", y);
    }

    public void field_centric(@NonNull BNO055IMU imu, com.qualcomm.robotcore.hardware.Gamepad gamepad) {
        if (gamepad.dpad_right || gamepad.dpad_left || gamepad.dpad_up || gamepad.dpad_down)
            field_centric_dpad(imu, gamepad);
        else field_centric_stick(imu, gamepad);
    }

    public void Elevator(boolean upMove, boolean downMove, double power) {
        telemetry.addData("mE ticks: ", mE.getCurrentPosition());

        if (gamepad2.dpad_up) {
            mE.setTargetPosition(mE.getCurrentPosition() + dEu);
            mE.setPower(1);
            mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (gamepad2.dpad_down) {
            mE.setTargetPosition(mE.getCurrentPosition() - (int) (mE.getCurrentPosition() * dEd));
            mE.setPower(0.7);
            mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else {
            if (gamepad2.y) {
                mE.setTargetPosition(2800);
                mE.setPower(1);
                mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (gamepad2.b) {
                mE.setTargetPosition(1990);
                mE.setPower(1);
                mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (gamepad2.x) {
                mE.setTargetPosition(1200);
                mE.setPower(1);
                mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (gamepad2.a) {
                mE.setTargetPosition(0);
                mE.setPower(1);
                mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
    }


    public void gripReset() {

        telemetry.addData("grip : ", sG.getPosition());
        telemetry.update();
        sG.setPosition(TARGET_RESET);
    }

    public void resetElevator() {
        mE.setTargetPosition(0);
        mE.setPower(1);
        mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void grip() {
        telemetry.addData("grip : ", sG.getPosition());
        telemetry.update();
        if (gamepad2.right_bumper) {
            sG.setPosition(TARGET_OPEN);
        }
        if (gamepad2.left_bumper) {
            sG.setPosition(TARGET_CLOSE);
        }
    }
}





/*
package OpModes;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import SubSystems.Field_Centric;
import SubSystems.cGamepad;


@TeleOp(name="Final_TeleOp", group = "Final_OpModes")
@Config
public class Final_TeleOp extends OpMode
{
    // DcMotors
    private DcMotor mFL = null; // Front Left Dc Motor
    private DcMotor mFR = null; // Front Right Dc Motor
    private DcMotor mBL = null; // Back Left Dc Motor
    private DcMotor mBR = null; // Back Right Dc Motor
    private DcMotor mE = null; // Elevator DcMotor

    // servos
    private Servo sG = null; // Servo Pinch

    private BNO055IMU imu = null;
    cGamepad m1 = null;

    public static double TARGET_RESET = 0.0;
    public static double TARGET_OPEN = 0.5;
    public static double TARGET_CLOSE = 0;

    public static int dEu = 166;
    public static double dEd = 0.2;
    @Override
    public void init()
    {
        m1 = new cGamepad(gamepad1); // one push GamePad

        // Make sure your ID's match your configuration
        mFL = hardwareMap.get(DcMotor.class, "mFL");
        mFR = hardwareMap.get(DcMotor.class, "mFR");
        mBL = hardwareMap.get(DcMotor.class, "mBL");
        mBR = hardwareMap.get(DcMotor.class, "mBR");
        mE = hardwareMap.get(DcMotor.class, "mE");

        sG = hardwareMap.get(Servo.class, "Grip");

        // DcMotors directions
        mFL.setDirection(DcMotor.Direction.REVERSE);
        mFR.setDirection(DcMotor.Direction.FORWARD);
        mBL.setDirection(DcMotor.Direction.REVERSE);
        mBR.setDirection(DcMotor.Direction.FORWARD);
        mE.setDirection(DcMotor.Direction.REVERSE);

        mE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // servo direction
        sG.setDirection(Servo.Direction.FORWARD);

        // DcMotors Zero Power Behavior
        mFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        // gyro calibration
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }

    @Override
    public void init_loop()
    {
        gripReset();
        resetElevator();

    }

    @Override
    public void start() {}

    @Override
    public void loop()
    {

        m1.update();
        field_centric(imu, gamepad1);
        Elevator(gamepad2.y, gamepad2.a, 1);
        grip();
        telemetry.update();

    }

    @Override
    public void stop()
    {
    }

    public int run_Mode_calc(boolean doit)
    {
        if (doit) {
            Field_Centric.Globals.run_Mode = Field_Centric.Globals.run_Mode + 1;
            Field_Centric.Globals.run_Mode = Field_Centric.Globals.run_Mode % 2;
        }

        return Field_Centric.Globals.run_Mode;
    }

    public static class Globals
    {
        public static int run_Mode = 0;
        public static int temp = 0;
        public static double offset = 0;
    }
    public void field_centric_offset(@NonNull BNO055IMU imu, com.qualcomm.robotcore.hardware.Gamepad gamepad)
    {
        double rx = gamepad.right_stick_x/2;

        double angle = -imu.getAngularOrientation().firstAngle;

        mFL.setPower((0 + 0 + rx));
        mBL.setPower((0 - 0 + rx));
        mFR.setPower((0 - 0 - rx));
        mBR.setPower((0 + 0 - rx));

        telemetry.addData("global offset: ", angle);
        Globals.offset = angle;
    }
    public void field_centric_stick(@NonNull BNO055IMU imu, com.qualcomm.robotcore.hardware.Gamepad gamepad)
    {
        double tt = gamepad.right_trigger/2;
        double y = -gamepad.left_stick_y; // Remember, this is reversed!
        double x = gamepad.left_stick_x; // Counteract imperfect strafing
        double rx = gamepad.right_stick_x;
        rx = (rx * tt) + (rx / 2);

        double angle = -imu.getAngularOrientation().firstAngle - Globals.offset;

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


        double angle = -imu.getAngularOrientation().firstAngle - Globals.offset;

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
        if(gamepad.right_stick_button) field_centric_offset(imu, gamepad);
        else if (gamepad.dpad_right || gamepad.dpad_left || gamepad.dpad_up || gamepad.dpad_down) field_centric_dpad(imu, gamepad);
        else field_centric_stick(imu, gamepad);
    }

    public void Elevator(boolean upMove, boolean downMove, double power)
    {
        telemetry.addData("mE ticks: " , mE.getCurrentPosition());

        if (gamepad2.dpad_up)
        {
            mE.setTargetPosition(mE.getCurrentPosition() + dEu);
            mE.setPower(1);
            mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if (gamepad2.dpad_down)
        {
            mE.setTargetPosition(mE.getCurrentPosition() - (int)(mE.getCurrentPosition()*dEd));
            mE.setPower(0.7);
            mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else {
            if (gamepad2.y) {
                mE.setTargetPosition(2800);
                mE.setPower(1);
                mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (gamepad2.b) {
                mE.setTargetPosition(1990);
                mE.setPower(1);
                mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (gamepad2.x) {
                mE.setTargetPosition(1200);
                mE.setPower(1);
                mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (gamepad2.a) {
                mE.setTargetPosition(0);
                mE.setPower(1);
                mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }
        }
    }


    public void gripReset()
    {

        telemetry.addData("grip : ", sG.getPosition());
        telemetry.update();
        sG.setPosition(TARGET_RESET);
    }

    public void resetElevator(){
        mE.setTargetPosition(0);
        mE.setPower(1);
        mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void grip()
    {
        telemetry.addData("grip : ", sG.getPosition());
        telemetry.update();
        if (gamepad2.right_bumper)
        {
            sG.setPosition(TARGET_OPEN);
        }
        if (gamepad2.left_bumper) {
            sG.setPosition(TARGET_CLOSE);
        }
    }
}


 */