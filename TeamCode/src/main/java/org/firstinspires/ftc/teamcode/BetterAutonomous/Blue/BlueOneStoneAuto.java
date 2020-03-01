package org.firstinspires.ftc.teamcode.BetterAutonomous.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CypherAutoMethods;

@Autonomous(name = "Blue 1 Stone Auto", group = "Stone auto")
public class BlueOneStoneAuto extends CypherAutoMethods {
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initializeIMU();
        initVision(this);
        Team team = Team.BLUE;
        detector.team = team;
        waitForStart();
        ElapsedTime runtime = new ElapsedTime();
        loadingAuto(team, 1);
        double time = runtime.seconds();
        while(opModeIsActive()) {
            telemetry.addData("time taken", time);
            telemetry.update();
        }
    }
}
