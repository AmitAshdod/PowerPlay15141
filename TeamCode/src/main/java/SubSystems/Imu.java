package SubSystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Imu {

    public static BNO055IMU imu = null;

    HardwareMap hardwareMap;
    Telemetry telemetry;

    public Imu(HardwareMap hardwareMap , Telemetry telemetry)
    {

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

    }
}
