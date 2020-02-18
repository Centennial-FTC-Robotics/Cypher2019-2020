package org.firstinspires.ftc.teamcode.BetterAutonomous.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.CypherAutoMethods;

@Autonomous(name = "Blue Top Move Foundation Left", group = "Blue Bridge Auto")
public class BlueBottomMoveFoundation extends CypherAutoMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initEverything();

        waitForStart();
        getFoundation(1, Side.WALL);


    }
}
