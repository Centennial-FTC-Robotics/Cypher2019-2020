package org.firstinspires.ftc.teamcode.BetterAutonomous.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CypherAutoMethods;

@Autonomous(name = "Red Loading Top Park", group = "Red Loading Auto")
public class RedLoadingTopPark extends CypherAutoMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initEverything();

        waitForStart();
            /* Assuming the foundation claws are facing away the foundation,
            the robot needs to move 12 to the right and 20 forwards since
            it needs to park top in this path
             */

        testAutoMove(20, 0);
        testAutoMove(0, -12);

    }
}
