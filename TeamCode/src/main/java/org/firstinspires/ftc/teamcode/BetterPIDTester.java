package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class BetterPIDTester extends CypherAutoMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();
        betterSelfCorrectStrafe(22.75 * 5, 0);
    }
}
