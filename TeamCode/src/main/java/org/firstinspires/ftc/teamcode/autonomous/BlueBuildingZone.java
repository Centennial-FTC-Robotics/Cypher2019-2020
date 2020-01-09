package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CypherAutoMethods;

@Autonomous(name = "Blue Team Foundation", group = "Auto")
public class BlueBuildingZone extends CypherAutoMethods {

    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initEverything();

        waitForStart();
        currentPos.setLocation(6, 5); // set start point
        dir = 90;
        buildingAuto("blue");
    }
}
