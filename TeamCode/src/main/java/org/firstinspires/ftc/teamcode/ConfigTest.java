package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Config Tester", group = "Test")
public class ConfigTest extends CypherMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("0", "back right");
            telemetry.addData("1", "up right");
            telemetry.addData("2", "up left");
            telemetry.addData("3", "back left");
            telemetry.update();

            leftDown.setPower(.3);
            rightDown.setPower(.3);
            //leftUp.setPower(.3);

            }
        }
    }

