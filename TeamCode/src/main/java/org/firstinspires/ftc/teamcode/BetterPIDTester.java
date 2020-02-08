package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class BetterPIDTester extends  CypherAutoMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();
        try {
            testPIDThingy(10,0);
        } catch (StopException e) {
            stopEverything();
        }
    }
}
