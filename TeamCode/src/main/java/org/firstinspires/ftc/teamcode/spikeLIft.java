package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class spikeLIft extends LinearOpMode {
    private Servo spikeLIft = null;
    @Override
    public void runOpMode() {
        spikeLIft = hardwareMap.get(Servo.class,"spikeLift");

        waitForStart();
        while (opModeIsActive()) {
            spikeLift();

        }

    }
    public void spikeLift() {
        spikeLIft.setPosition(0.6);


    }
}
