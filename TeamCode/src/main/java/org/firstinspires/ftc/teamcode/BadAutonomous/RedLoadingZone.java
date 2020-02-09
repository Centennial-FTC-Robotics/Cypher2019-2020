package org.firstinspires.ftc.teamcode.BadAutonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CypherAutoMethods;

@Autonomous(name = "Red Team Loading Zone Left", group = "Auto")
public class RedLoadingZone extends CypherAutoMethods {

    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initEverything();


        waitForStart();

        currentPos.setLocation(6, 2);
        dir = 180;
        actualAuto(Team.RED, Side.WALL, 1);


    }
}
