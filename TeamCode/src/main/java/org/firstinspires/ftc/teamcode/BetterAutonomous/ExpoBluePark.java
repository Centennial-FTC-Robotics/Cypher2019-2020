package org.firstinspires.ftc.teamcode.BetterAutonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CypherAutoMethods;
@Autonomous
public class ExpoBluePark extends CypherAutoMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initEverything();
        waitForStart();
        testAutoMove(0,-20);
    }
}
