package org.firstinspires.ftc.teamcode.BetterAutonomous.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CypherAutoMethods;

@Autonomous(name = "Blue Building Top Park", group = "Blue Building Auto")
public class BlueBuildingTopPark extends CypherAutoMethods {
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initEverything();

        waitForStart();
            /* Assuming the foundation claws are facing the foundation,
            the robot needs to move 12 to the left and 20 backwards since
            it needs to park top in this path
             */
        testAutoMove(20, 0);
        testAutoMove(0, 12);

    }
}
