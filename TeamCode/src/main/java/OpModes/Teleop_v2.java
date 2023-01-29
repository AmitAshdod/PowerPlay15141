
package OpModes;


import androidx.annotation.NonNull;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


import SubSystems.Field_Centric;
import SubSystems.cGamepad;
@Disabled
@TeleOp(name="Teleop_v2")

public class Teleop_v2 extends OpMode
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

    public static double TARGET_RESET = 0;
    public static double TARGET_OPEN = 0.8;
    public static double TARGET_CLOSE = 0.2;

    int minLevel = 1225, midLevel = 2020, highLevel = 2850, target = 0;
    enum LEVELS {MIN, MID, HIGH};

    LEVELS currentLevel = LEVELS.HIGH;

    boolean catched = false, unreleased = true;
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
    }

    public void field_centric_stick(@NonNull BNO055IMU imu, com.qualcomm.robotcore.hardware.Gamepad gamepad)
    {
        double tt = gamepad.right_trigger/2;
        double y = -gamepad.left_stick_y; // Remember, this is reversed!
        double x = gamepad.left_stick_x; // Counteract imperfect strafing
        double rx = gamepad.right_stick_x;

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
        if(gamepad2.y)
        {
            currentLevel = LEVELS.HIGH;
        }
        else if(gamepad2.b)
        {
            currentLevel = LEVELS.MID;
        }
        else if(gamepad2.a)
        {
            currentLevel = LEVELS.MIN;
        }


        switch (currentLevel)
        {
            case MIN:
                target = minLevel;
                break;
            case MID:
                target = midLevel;
                break;
            case HIGH:
                target = highLevel;
                break;
        }

        if(gamepad1.right_bumper || catched && !gamepad1.left_bumper)
        {
            sG.setPosition(TARGET_CLOSE);

            catched = true;
            if(gamepad1.right_bumper || !unreleased)
            {
                unreleased = false;
                mE.setTargetPosition(target);
                mE.setPower(1);
                mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }

        if(gamepad1.left_bumper)
        {
            unreleased = true;
            catched = false;
            resetElevator();
        }

        telemetry.addData("grip : ", sG.getPosition());

        telemetry.addData("mE ticks: " , mE.getCurrentPosition());
    }


    public void resetElevator(){
        mE.setTargetPosition(0);
        mE.setPower(1);
        mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sG.setPosition(TARGET_OPEN);
    }

}
