package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@Autonomous(name="Autonomous", group="Autonomous")
public class servoTest extends LinearOpMode {
    CRServo leftServo;
    CRServo rightServo;

    @Override
    public void runOpMode() throws InterruptedException {
    leftServo = hardwareMap.crservo.get("leftservo");
    rightServo = hardwareMap.crservo.get("rightservo");

    leftServo.setDirection(CRServo.Direction.REVERSE);

    waitForStart();

    while(opModeIsActive()) {
        /*boolean servoOn = gamepad1.a;

        if(servoOn) {*/
            leftServo.setPower(1);
            rightServo.setPower(1);
        //}

    }

    }


}
