package org.firstinspires.ftc.teamcode.BetterAutonomous.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CypherAutoMethods;

@Autonomous(name = "Blue Stone Auto Bridge", group = "Stone auto")
public class BlueBridgeStoneAuto extends CypherAutoMethods {
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initializeIMU();
        initVision(this);
        Team team = Team.BLUE;
        detector.team = team;
        waitForStart();
        ElapsedTime runtime = new ElapsedTime();
        loadingAuto(team, 2);
        while (opModeIsActive()) {
            telemetry.addData("time taken", runtime.seconds());
            telemetry.update();
        }
    }
}
