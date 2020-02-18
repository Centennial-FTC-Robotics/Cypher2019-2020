package org.firstinspires.ftc.teamcode.BetterAutonomous.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.CypherAutoMethods;

@Autonomous(name = "Red Stone Auto Bridge", group = "Stone auto")
public class RedBridgeStoneAuto extends CypherAutoMethods {
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initEverything();

        waitForStart();
        loadingAuto("red", 1);
    }
}
