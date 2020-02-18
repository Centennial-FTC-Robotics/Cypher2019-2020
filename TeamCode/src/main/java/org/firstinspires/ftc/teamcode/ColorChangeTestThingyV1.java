package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class ColorChangeTestThingyV1 extends CypherMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();
        while (opModeIsActive()) {
            changeColor(RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE);
            telemetry.addData("color is", "dark blue");
            telemetry.update();
            waitMili(5000);
            changeColor(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            telemetry.addData("color is", "yellow");
            telemetry.update();
            waitMili(5000);

        }
    }
}
