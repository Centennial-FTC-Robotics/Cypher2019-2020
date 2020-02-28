package org.firstinspires.ftc.teamcode.BetterAutonomous.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CypherAutoMethods;

@Autonomous(name = "Red 1 Stone Auto Bridge", group = "Stone auto")
public class RedBridgeOneStoneAuto extends CypherAutoMethods {
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initializeIMU();
        initVision(this);
        Team team = Team.RED;
        detector.team = team;
        waitForStart();
        ElapsedTime runtime = new ElapsedTime();
        loadingAuto(team, 1);
        while(opModeIsActive()) {
            telemetry.addData("time taken", runtime.seconds());
            telemetry.update();
        }
    }
}
