package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Config Tester", group = "Test")
public class ConfigTest extends CypherMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        resetEncoders();
        waitForStart();
        arm.setPosition(0.35);
        while(opModeIsActive()) {
            telemetry.addData("pos", arm.getPosition());
            telemetry.update();
        }

    }
}


