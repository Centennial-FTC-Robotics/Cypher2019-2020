package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CypherAutoMethods;

@Autonomous(name = "Blue Team Loading Zone", group = "Auto")
public class BlueLoadingZone extends CypherAutoMethods {

    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initEverything();

        waitForStart();

        currentPos.setLocation(1, 2);
        loadingAuto("blue");
    }
}
