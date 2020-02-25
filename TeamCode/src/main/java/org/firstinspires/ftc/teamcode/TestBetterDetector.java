package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class TestBetterDetector extends CypherAutoMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initVision(this);
        detector.team = Team.BLUE;
        waitForStart();

        /*
        while (!opModeIsActive() && !isStopRequested()) {
            detector.orderStones();
        }
        int[] skystonePositions = detector.getSkystonePositions();
        telemetry.addData("first skystone", skystonePositions[0]);
        telemetry.addData("second skystone", skystonePositions[1]);
        telemetry.update();

        moveToStone(skystonePositions[0], Team.BLUE);

         */
        while(opModeIsActive()) {
            detector.debugStuff();
        }
    }
}
