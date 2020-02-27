package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class TestBetterDetector extends CypherAutoMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initVision(this);
        detector.team = Team.RED;
        waitForStart();
        getInPos(Team.RED);
        while(opModeIsActive()) {
            int[] skystonePositions = detector.getSkystonePositions();
            telemetry.addData("first skystone", skystonePositions[0]);
            telemetry.addData("second skystone", skystonePositions[1]);
            telemetry.update();
        }


    }
}
