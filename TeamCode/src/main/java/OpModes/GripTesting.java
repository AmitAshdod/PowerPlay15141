package OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;



import SubSystems.cGamepad;


@TeleOp(name = "GripTesting", group = "TeleOp")

@Config
public class GripTesting extends LinearOpMode {

    cGamepad m2 = new cGamepad(gamepad2);

    Servo Grip;

    public static double TARGET_RESET = 0;
    public static double TARGET_OPEN = 0.8;
    public static double TARGET_CLOSE = 0.2;
    public static double TARGET_HALFWAY = 0.25;


    public void runOpMode() throws InterruptedException {

        Grip = hardwareMap.get(Servo.class, "Grip");
        //Grip.setPosition(TARGET_RESET);

        waitForStart();
        if (isStopRequested()) return;
        while ( opModeIsActive()){

                telemetry.addData("grip : ", Grip.getPosition());
                telemetry.update();


            while (opModeIsActive()) {

                telemetry.addData("grip : ", Grip.getPosition());
                telemetry.update();
                if (gamepad2.y) {

                    Grip.setPosition(TARGET_OPEN);
                }

                if (gamepad2.x) {

                    Grip.setPosition(TARGET_HALFWAY);
                }

                if (gamepad2.b) {

                    Grip.setPosition(TARGET_CLOSE);
                }

            }
        }
    }

}
