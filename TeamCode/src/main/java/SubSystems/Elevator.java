package SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Elevator {

    DcMotor mE = null;

    HardwareMap hardwareMap;
    Telemetry telemetry;


    int minLevel = 1200, midLevel = 1990, highLevel = 2800, target = 0;
    double power = 1;

    public Elevator(HardwareMap hardwareMap, Telemetry telemetry)
    {
        mE = hardwareMap.get(DcMotor.class, "mE");
        mE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mE.setTargetPosition(target);

        }

        public void highRod(){

        mE.setTargetPosition(highLevel);
        mE.setPower(power);
        mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
        public void midRod(){

        mE.setTargetPosition(midLevel);
        mE.setPower(power);
        mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

        public void lowRod(){

        mE.setTargetPosition(minLevel);
        mE.setPower(power);
        mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

        public void resetElevator(){

        mE.setTargetPosition(target);
        mE.setPower(power);
        mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        public int updateElevator(int pos){

        pos = mE.getCurrentPosition();

        return pos;
        }
}

