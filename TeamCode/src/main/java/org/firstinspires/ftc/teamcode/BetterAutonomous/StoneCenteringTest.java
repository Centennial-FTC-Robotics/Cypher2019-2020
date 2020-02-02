package org.firstinspires.ftc.teamcode.BetterAutonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.CypherAutoMethods;
import org.firstinspires.ftc.teamcode.StopException;

@Autonomous(name = "Center Test Just Print", group = "Test")
public class StoneCenteringTest extends CypherAutoMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
        }
        waitForStart();
        try {
            skystonePrintPls(1);
        } catch (StopException e) {
            stopEverything();
        }


    }

}
