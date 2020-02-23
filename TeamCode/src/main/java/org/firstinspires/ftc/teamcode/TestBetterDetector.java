package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class TestBetterDetector extends CypherAutoMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initVision(this);
        waitForStart();
        while(opModeIsActive()) {
            //detector.determineOrder();
            detector.debug();
            telemetry.update();
            int[] skystonePositions = detector.getSkystonePositions();
        }
    }
}
