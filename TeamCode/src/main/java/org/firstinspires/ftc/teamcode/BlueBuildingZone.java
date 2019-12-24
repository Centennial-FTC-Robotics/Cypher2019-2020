package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Team Foundation", group = "Auto")
public class BlueBuildingZone extends CypherMethods {

    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initEverything();

        waitForStart();
        currentPos.setLocation(6, 5); // set start point
        dir = 90;
        buildingAuto("blue");
    }
}