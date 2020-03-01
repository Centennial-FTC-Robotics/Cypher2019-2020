package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class reee extends  CypherMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        vSlideEncoder = 5;
        waitForStart();
        writeVSlideData();
        while (opModeIsActive()) {
            updateVSlideData();
            telemetry.addData("yes", vSlideEncoder);
            telemetry.update();
        }
    }
}
