package org.firstinspires.ftc.teamcode.BetterAutonomous.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CypherAutoMethods;


@Autonomous(name = "Red Building Bottom Park", group = "Red Building Auto")
public class RedBuildingBottomPark extends CypherAutoMethods {
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initEverything();

        waitForStart();
            /* Assuming the foundation claws are facing away from the foundation,
            the robot needs to move 12 to the right and 0 backwards since
            it needs to park top in this path
            */

        testAutoMove(0, 12);

    }
}
