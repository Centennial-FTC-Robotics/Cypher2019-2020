package org.firstinspires.ftc.teamcode.BadAutonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.CypherAutoMethods;
import org.firstinspires.ftc.teamcode.StopException;
@Disabled
@Autonomous(name = "red team 1 skystone", group = "Auto")
public class RedOneSkystone extends CypherAutoMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        try {
            initEverything();
        } catch (StopException e) {
            stopEverything();
        }
        waitForStart();

        currentPos.setLocation(1, 2);
        loadingAuto("red", 1);
    }
}
