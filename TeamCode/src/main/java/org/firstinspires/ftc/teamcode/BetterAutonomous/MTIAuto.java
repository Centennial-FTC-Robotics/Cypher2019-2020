package org.firstinspires.ftc.teamcode.BetterAutonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CypherAutoMethods;

@Autonomous(name = "Auto for MTI")
public class MTIAuto extends CypherAutoMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        int amount = 2;
        super.runOpMode();
        initializeIMU();
        Team team = Team.BLUE;
        waitForStart();
        double move = 0;


        //releaseIntake(); //DONT U DARE UNCOMMENT THIS LINE IT WILL BREAK THE SLIDES NO TOUCH THIS

        //let the intake thingy out
        controlIntakeMotors(0.4);
        double angle = getRotationDimension();
        int factor = -1;
        move += -(TILE_LENGTH * (1d / 2) + 4);
        move += -(TILE_LENGTH * (2d / 3) - 4);
        double angleOther = getRotationDimension();
        int[] skystonePos = {2,5};
        //does not clear the telemetry for the stone positions!1!!11!!
        telemetry.addData("first skystone", skystonePos[0]).setRetained(true);
        telemetry.addData("second skystone", skystonePos[1]).setRetained(true);
        telemetry.update();

        int otherFactor = 1;
        int idkWhatToCallThis;
        double dist;

        //we need to test out some values for this
        /* whats needed
        where will we tell the robot to go to see the first 2 or first 3 stones
        how far does the robot need to travel backwards to get to a stone
        how far does the robot need to strafe to knock others out of its way
        how far does the robot need to move to actually get the stone in the intake
         */
        int pos = 2;
        idkWhatToCallThis = 4 - pos;
        dist = otherFactor * (idkWhatToCallThis * 6);

        //Bill was here lol

        move +=  -(TILE_LENGTH * (2d / 3d) + 4);

        autoMove(0,move);
        controlIntakeMotors(0.7777777777777777777);
        autoMove(8, 0);
        autoMove(0, (TILE_LENGTH * (2d / 3d) + 8));
        dist += -10 * otherFactor;

        double distTravelled = dist;

        //now u can run the proper auto!!!11!!!

        turnAbsolute((angle + angleOther) / 2); //rotate back to original angle (hitting the stones messes it up a bit)
        controlIntakeMotors(0.3);
        autoMove(-(TILE_LENGTH * 3.7 + distTravelled) * factor, 0); //move to other side
        turnAbsolute(90); //turn
        //aquire foundation
        autoMove(-TILE_LENGTH * (2d / 3) + 4, 0);
        controlFoundation(FoundationState.DRAG);
        waitMilli(500);
        grabStone(ArmState.DROP);
        autoMove(TILE_LENGTH * 2, 0);
        controlIntakeMotors(-0.3);
        turnAbsolute(180);
        controlIntakeMotors(0);

        autoMove(-25, 0);
        //ok who needs foundation anymore
        controlFoundation(FoundationState.RELEASE);
        waitMilli(400);

        //2nd stone time
        if (amount == 2) {
            //blah blah blah idfk code works
            //maybe
            autoMove(findDistToSkystone(skystonePos[1]) + (TILE_LENGTH * 3.8), 0);

            autoMove(0, ((TILE_LENGTH * (2d / 3d) + 4) * 2) - 6);
            waitControlIntake(0.7777777777777777777);
            autoMove(8, 0);
            autoMove(0, -((TILE_LENGTH * (2d / 3d) + 10) * 2) - 6);


            grabStone(ArmState.PICK);
            controlIntakeMotors(0.3);

            autoMove(-(findDistToSkystone(skystonePos[1]) + (TILE_LENGTH * 3.5)), 0);
            grabStone(ArmState.DROP);

            turnRelative(90); //if it breaks prob thia line right here

            controlIntakeMotors(-0.3);
            waitMilli(300);
            controlIntakeMotors(0);

            autoMove(0, -TILE_LENGTH * 1.5);
        } else {
            autoMove(40, 0);
        }
    }
}

