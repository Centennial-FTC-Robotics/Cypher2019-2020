package org.firstinspires.ftc.teamcode.BetterAutonomous.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CypherAutoMethods;

@Autonomous(name = "Red Top Move Foundation Left", group = "Red Bridge Auto")
public class RedBottomMoveFoundation extends CypherAutoMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initEverything();

        waitForStart();
        getFoundation(-1, Side.WALL);


    }
}
