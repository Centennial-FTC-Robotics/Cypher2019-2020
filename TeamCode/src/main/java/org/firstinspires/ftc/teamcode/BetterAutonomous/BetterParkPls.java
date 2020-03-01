package org.firstinspires.ftc.teamcode.BetterAutonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.CypherAutoMethods;

@Autonomous(name = "General Park", group = "Park")
public class BetterParkPls extends CypherAutoMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initEverything();

        waitForStart();
        autoMove(0, 16);

    }
}
