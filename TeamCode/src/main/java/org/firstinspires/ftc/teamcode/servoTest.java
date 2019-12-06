package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="servoTest", group="Test")
public class servoTest extends LinearOpMode {
    CRServo leftServo;
    CRServo rightServo;

    @Override
    public void runOpMode() throws InterruptedException {


    waitForStart();

    while(opModeIsActive()) {
        boolean in = gamepad1.a;
        boolean out = gamepad1.b;
        boolean reset = gamepad1.y;
             if (in) {
                leftServo.setPower(1);
                rightServo.setPower(1);
            } else if (out) {
                leftServo.setPower(-1);
                rightServo.setPower(-1);
            } else if  (reset) {
                 leftServo.setPower(0);
                 rightServo.setPower(0);
             }



    }

    }


}
