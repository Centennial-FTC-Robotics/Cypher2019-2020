package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Red Team Loading Zone", group = "Auto")
public class RedLoadingZone extends CypherMethods {

    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initEverything();

        waitForStart();

        currentPos.setLocation(6,2);
        dir = -90;
        loadingAuto("red");
    }
}
