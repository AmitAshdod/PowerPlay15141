package SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Grip {

    Servo sG;

    boolean Caught = false;
    boolean released = true;

    public static double TARGET_RESET = 0;
    public static double TARGET_OPEN = 0.8;
    public static double TARGET_CLOSE = 0.2;


    HardwareMap hardwareMap;
    Telemetry telemetry;

    public Grip(Telemetry telemetry, HardwareMap hardwareMap) {

        sG = hardwareMap.get(Servo.class, "sG");
        sG.setPosition(TARGET_RESET);

    }

    public void gripClose() {

        Caught = true;
        released = false;

        if (Caught) {

            sG.setPosition(TARGET_CLOSE);


        }
    }

     public void gripOpen() {

        if(Caught == false && released == true) {
            sG.setPosition(TARGET_OPEN);
        }
    }
}

