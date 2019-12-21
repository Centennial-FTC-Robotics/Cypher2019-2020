package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Blue Team Quarry", group = "Auto")
public class BlueLoadingZone extends CypherMethods {

    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initEverything();

        waitForStart();

        currentPos.setLocation(1,2);
        loadingAuto("blue");
    }
}
