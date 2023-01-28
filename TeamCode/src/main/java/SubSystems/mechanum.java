

package SubSystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="FUCKI DUCK2323", group="Iterative Opmode")

public class mechanum extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //DC motors
    private DcMotor mFL = null;
    private DcMotor mFR = null;
    private DcMotor mBL = null;
    private DcMotor mBR = null;

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


        mFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


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


        double y = Range.clip(-gamepad1.left_stick_y, -1.0, 1.0); // Remember, this is reversed!
        double x = Range.clip(gamepad1.left_stick_x, -1.0, 1.0);
        double rx = Range.clip(gamepad1.right_stick_x, -1.0, 1.0);


        mFL.setPower(y + x + rx);
        mBL.setPower(y - x + rx);
        mFR.setPower(y - x - rx);
        mBR.setPower(y + x - rx);



    }

    @Override
    public void stop()
    {
    }

}
