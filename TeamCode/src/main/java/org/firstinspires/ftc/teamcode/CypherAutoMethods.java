package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class CypherAutoMethods extends CypherMethods {

    public final Tile currentPos = new Tile(0, 0); //always start here
    private final Tile redFoundation = new Tile(5, 5.5);
    private final Tile redBuildSite = new Tile(6, 5.5);
    private final Tile redQuarry = new Tile(5.5, 2);
    private final Tile redBridge = new Tile(5.5, 3.5);
    private final Tile blueFoundation = new Tile(2, 5.5);
    private final Tile blueBuildSite = new Tile(1, 5.5);
    private final Tile blueQuarry = new Tile(1.5, 2);
    private final Tile blueBridge = new Tile(1.5, 3.5);
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
    }

    void emergencyMove(String side, String color) {
        //no encoders, loading zone, building zone, red, blue
        ElapsedTime timer = new ElapsedTime();
        double factor;

        if (side.equals("loading")) {
            if(color.equals("red")) factor = 1;
            else factor = -1;
        }  else {
            if(color.equals("red")) factor = -1;
            else factor = 1;

        }
        telemetry.addData("EMERGENCY", "ROBOT DOES NOT WORK NORMALLY");
        telemetry.update();
        timer.reset();
        do {
            setStrafeMotors(-0.4*factor, 0.4*factor);
        } while (timer.seconds() < 2);
        setDriveMotors(0);
    }

    public void buildingAuto(String side) {
        int factor = 1;
        switch (side) {
            case "red":
                factor = 1;
                break;
            case "blue":
                factor = -1;
        }
        Tile oldPos;
        moveToPos(currentPos.getX() - .5 * factor, currentPos.getY(), dir); //move forward a small bit
        turnRelative(180); //turn around so we can pick up foundation
        dir = 90 * factor;
        if (factor == 1)
            moveToPos(redFoundation, dir); //go to foundation
        else
            moveToPos(blueFoundation, dir);
        waitMoveFoundation(1);

        if (factor == 1)
            moveToPos(redBuildSite, dir);
        else
            moveToPos(blueBuildSite, dir);
        waitMoveFoundation(-1);
        moveFoundation(0);
        if (factor == 1)
            moveToPos(redQuarry, dir); //go to red quarry
        else
            moveToPos(blueQuarry, dir);
        turnRelative(-90 * factor);
        dir = 180;

        for (int i = 0; i < 2; i++) { //repeat twice for 2 skystones

            skystoneFindPls(factor); //center with skystone
            currentPos.add(0, -convertInchToTile(convertEncoderToInch(getPos()))); //find how far we travelled to find skystone
            oldPos = new Tile(currentPos);
            moveToPos(currentPos.getX() - convertInchToTile(factor), currentPos.getY(), dir); //move to be right behind/infront/whatever of skystone

            //pick up skystone and move into it
            controlIntakeServos(1);
            testAutoMove(2, 0);
            currentPos.add(0, convertInchToTile(-2));

            moveToPos(currentPos.getX() + factor, currentPos.getY(), dir); //move a bit to prevent hitting the neutral bridge
            moveToPos(currentPos.getX(), currentPos.getY() + 2, dir); //move to other side

            turnRelative(-90 * factor); //turn to release skystone and not have it in the way
            dir = -90 * factor;
            waitControlIntake(-1); //release skystone
            controlIntakeServos(0);
            turnRelative(90 * factor); //turn back
            dir = 180;
            if (i == 0) //if that was the first skystone move back to where we got the first one to look for second ome
                moveToPos(oldPos, dir);
        }
        if (factor == 1)
            moveToPos(redBridge, dir); //go to red bridge
        else
            moveToPos(blueBridge, dir); //or blue bridge
    }

    protected void loadingAuto(String side) {
        int factor = 1;
        switch (side) {
            case "red":
                factor = 1;
                break;
            case "blue":
                factor = -1;
                break;
        }

        testAutoMove(2, 0);
        currentPos.add(convertInchToTile(2) * factor, 0);
        turnRelative(-90 * factor);
        dir = 180;
        for (int i = 0; i < 2; i++) {
            skystoneFindPls(factor);
            currentPos.add(0, -convertInchToTile(convertEncoderToInch(getPos()))); //find how far we travelled to find skystone
            Tile oldPos = new Tile(currentPos);
            moveToPos(currentPos.getX() - convertInchToTile(factor), currentPos.getY(), dir); //move to be right behind/infront/whatever of skystone

            waitControlIntake(1);
            testAutoMove(2, 0);
            currentPos.add(convertInchToTile(2), 0);

            moveToPos(currentPos.getX() + factor, currentPos.getY(), dir); //move a bit to prevent hitting the neutral bridge
            moveToPos(currentPos.getX(), blueBridge.getY() + 1.5, dir); //move to other side

            turnRelative(-90 * factor); //turn to spit out block w/o it getting in way
            dir = -90 * factor; //change dir
            waitControlIntake(-1); //spit it out

            turnRelative(180);
            dir *= -1;
            if (i == 0) { //if its the first skystone move foundation
                if (factor == 1) {
                    moveToPos(redFoundation, dir);

                } else {
                    moveToPos(blueFoundation, dir);
                }
                waitMoveFoundation(1);
                if (factor == 1) {
                    moveToPos(redBuildSite, dir);
                } else {
                    moveToPos(blueBuildSite, dir);
                }
                waitMoveFoundation(-1);
                moveFoundation(0);
            }

            if (i == 0) {
                moveToPos(oldPos, dir);
            }

            turnRelative(90 * factor);
            dir = 180;
        }
        if (factor == 1) {
            moveToPos(redBridge, dir);
        } else {
            moveToPos(blueBridge, dir);
        }

    }

    private void moveToPos(double x2, double y2, int dir) {
        moveToPos(new Tile(x2, y2), dir);
    }

    private void moveToPos(Tile end, int dir) {
        double[] move = getDist(currentPos, end, dir);
        testAutoMove(move[0], move[1]);
        currentPos.setLocation(end);
    }

    private void waitControlIntake(double power) {
        ElapsedTime time = new ElapsedTime();
        controlIntakeServos(power);
        while (time.milliseconds() < 200) ;
    }

    private void waitMoveFoundation(double power) {
        ElapsedTime time = new ElapsedTime();
        moveFoundation(power);
        while (time.milliseconds() < 200) ;
    }
    private double[] getDist(Tile start, Tile end, int dir) {
        double forward, left;
        /* Note: Code to apply for any given angle. Not sure if it works, so it's commented. Let's tests.
        if (dir >= 0) {
            directionNew = 360 - (dir - 90);
        }
        else {
            directionNew = -dir + 90;
        }
        int startHorizontalAdjust = start.getX()*Math.cos(directionNew) + start.getY()*Math.sin(directionNew);
        int startVerticalAdjust = -start.getX()*Math.sin(directionNew) + start.getY()*Math.cos(directionNew);
        int finalHorizontalAdjust = end.getX()*Math.cos(directionNew) + end.getY()*Math.sin(directionNew);
        int finalVerticalAdjust = -end.getX()*Math.sin(directionNew) + end.getY()*Math.cos(directionNew);
        foward = finalVerticalAdjust - startVerticalAdjust;
        left = finalHorizontalAdjust - startHorizontalAdjust;
        return new double[]{tilesToInch(forward), tilesToInch(left)};
         */
        switch (dir) {
            case 90:
                forward = end.getX() - start.getX();
                left = end.getY() - start.getY();
                break;
            case 180:
                forward = start.getY() - end.getY();
                left = end.getX() - start.getX();
                break;
            case -90:
                forward = start.getX() - end.getX();
                left = end.getY() - start.getY();
                return new double[]{tilesToInch(forward), tilesToInch(left)};
            default:
                forward = end.getY() - start.getY();
                left = end.getX() - start.getX();
                break;
        }
        return new double[]{tilesToInch(forward), tilesToInch(left)};
    }


}
