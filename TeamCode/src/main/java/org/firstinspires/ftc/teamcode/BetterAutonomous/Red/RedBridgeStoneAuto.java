package org.firstinspires.ftc.teamcode.BetterAutonomous.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CypherAutoMethods;

@Autonomous(name = "Red Stone Auto Bridge", group = "Stone auto")
public class RedBridgeStoneAuto extends CypherAutoMethods {
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initializeIMU();
        initVision(this);
        Team team = Team.RED;
        detector.team = team;
        waitForStart();
        ElapsedTime runtime = new ElapsedTime();
        loadingAuto(team, 2);
        double time = runtime.seconds();
        while(opModeIsActive()) {
            telemetry.addData("time taken", time);
            telemetry.update();
        }
    }
}
