package org.firstinspires.ftc.teamcode.BadAutonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.CypherAutoMethods;

@Disabled
@Autonomous(name = "Red Loading/Blue Building EMERG", group = "Auto")
public class RedLoadingBlueBulding extends CypherAutoMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initEverything();

        waitForStart();
        emergRedLoading();
    }
}
