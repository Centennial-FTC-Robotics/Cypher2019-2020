package org.firstinspires.ftc.teamcode.BetterAutonomous.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.CypherAutoMethods;

@Autonomous(name = "Blue Stone Auto Bridge", group = "Stone auto")
public class BlueBridgeStoneAuto extends CypherAutoMethods {
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initEverything();

        waitForStart();
        currentPos.setLocation(2,1);
        dir = 0;
        loadingAuto("blue", 1);
    }
}
