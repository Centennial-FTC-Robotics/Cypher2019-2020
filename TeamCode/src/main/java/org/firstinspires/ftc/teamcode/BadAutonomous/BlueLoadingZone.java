package org.firstinspires.ftc.teamcode.BadAutonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.CypherAutoMethods;

@Disabled
@Autonomous(name = "Blue Team Loading Zone", group = "Auto")
public class BlueLoadingZone extends CypherAutoMethods {

    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initEverything();

        waitForStart();

        currentPos.setLocation(1, 2);
        loadingAuto("blue", 2);
    }
}
