

package OpModes;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Disabled
@TeleOp(name="FUCKI DUCK2", group="Iterative Opmode")

public class BasicOpMode_Iterative extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //DC motors
    private DcMotor mFL = null;
    private DcMotor mFR = null;
    private DcMotor mBL = null;
    private DcMotor mBR = null;

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;

    @Override
    public void init()
    {
        telemetry.addData("Status", "Initialized");

        mFL = hardwareMap.get(DcMotor.class, "mFL");
        mFR = hardwareMap.get(DcMotor.class, "mFR");
        mBL = hardwareMap.get(DcMotor.class, "mBL");
        mBR = hardwareMap.get(DcMotor.class, "mBR");

        mFL.setDirection(DcMotor.Direction.REVERSE);
        mFR.setDirection(DcMotor.Direction.FORWARD);
        mBL.setDirection(DcMotor.Direction.REVERSE);
        mBR.setDirection(DcMotor.Direction.FORWARD);

        /*
        mFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        */
        //IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        while (!imu.isGyroCalibrated())
        {
            try {
                sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();
        //IMU


        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop()
    {
    }

    @Override
    public void start()
    {
        runtime.reset();
    }

    @Override
    public void loop()
    {
        /* tank robot
        double leftPower;
        double rightPower;

        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

        mFL.setPower(leftPower);
        mFR.setPower(rightPower);
        mBL.setPower(leftPower);
        mBR.setPower(rightPower);
        */

        double y = Range.clip(-gamepad1.left_stick_y, -1.0, 1.0); // Remember, this is reversed!
        double x = Range.clip(gamepad1.left_stick_x, -1.0, 1.0);
        double rx = Range.clip(gamepad1.right_stick_x, -1.0, 1.0);

        //double y = -gamepad1.left_stick_y;
        //double x = gamepad1.left_stick_x;
        //double rx = gamepad1.right_stick_x;

        //x = Math.round(x * 1000.0) / 1000.0;
        //y = Math.round(y * 1000.0) / 1000.0;
        //rx = Math.round(rx * 1000.0) / 1000.0;

        double output_xy[] = bydir_movment(x, y);

        mecanum_movement(output_xy[0], output_xy[1], rx);



    }

    @Override
    public void stop()
    {
    }

    private void mecanum_movement(double x, double y, double rx)
    {
        mFL.setPower(y + x + rx);
        mBL.setPower(y - x + rx);
        mFR.setPower(y - x - rx);
        mBR.setPower(y + x - rx);
    }

    private double goystick_angle(double x, double y)
    {
        double angle = Math.toDegrees(Math.atan(x/y));
        if(y < 0 && x == 0)
        {
            angle = 180;
        }
        else if (y < 0 && x < 0)
        {
            angle = 180 - angle;
        }
        else if (y < 0 && x > 0)
        {
            angle = -(180 + angle);
        }
        else if (y > 0 && (x < 0 || x > 0))
        {
            angle *= -1;
        }


        return angle;
    }

    private double calc_bydir_angle(double target_deg, double global_angle)
    {
        double output_deg = target_deg - global_angle;
        if (output_deg > 180)
        {
            output_deg = -(output_deg % 180);
        }
        else if (output_deg < -180)
        {
            output_deg = -(output_deg % -180);
        }

        return output_deg;
    }

    private double[] calc_bydir_xy(double angle, double power)
    {
        double[] output_xy = new double[2];

        int angel_sign = Integer.signum((int)angle);
        double temp_angle = angel_sign * ((angel_sign * 180) - angle);

        double x = 0;
        double y = 0;

        if (temp_angle == 90)
        {
            x = power;
            y = 0;
        }
        else
        {
            x = Math.sin(temp_angle) * power;
            y = Math.cos(temp_angle) * power;
        }

        if (angel_sign > 0)
        {
            x = -x;
        }
        if (angle > 90 || angle < -90)
        {
            y = -y;
        }

        output_xy[0] = x;//Math.round(x * 100.0) / 100.0;
        output_xy[1] = y;//Math.round(y * 100.0) / 100.0;
        return output_xy;
    }

    private double[] bydir_movment(double target_x, double target_y)
    {
        double[] output_xy;

        double target_power = Math.sqrt(Math.pow(target_x, 2) + Math.pow(target_y, 2));
        double target_deg = goystick_angle(target_x, target_y);

        double output_deg = calc_bydir_angle(target_deg, getAngle());

        target_power = Range.clip(target_power, 0, 1.0);

        output_xy = calc_bydir_xy(output_deg, target_power);

        telemetry.addData("global heading: ", getAngle());
        telemetry.addData("x: ", target_x);
        telemetry.addData("y: ", target_y);
        telemetry.addData("target_power: ", target_power);
        telemetry.addData("\ntarget_deg: ", target_deg);
        telemetry.addData("output_deg: ", output_deg);
        telemetry.addData("\noutput_x: ", output_xy[0]);
        telemetry.addData("output_y: ", output_xy[1]);
        telemetry.addData("output_recalc_angle: ", goystick_angle(output_xy[0], output_xy[1]));
        telemetry.update();

        return output_xy;
    }

    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double Angle = angles.firstAngle - lastAngles.firstAngle;

        return Angle;
    }
}
