package org.firstinspires.ftc.teamcode;

public class UnusedMethods {
    /*
    private double getRotationDimension(char dimension) {
        switch (Character.toUpperCase(dimension)) {
            case 'X':
                return AngleUnit.normalizeDegrees(rawDimension('X') - initialPitch);
            case 'Y':
                return AngleUnit.normalizeDegrees(rawDimension('Y') - initialRoll);
            case 'Z':
                return AngleUnit.normalizeDegrees(rawDimension('Z') - initialHeading);
        }

        return 0;
    }
    double getArmPos() {
        return arm.getPosition();
    }
    private double rawDimension(char dimension) {
        orientationUpdate();
        switch (dimension) {
            case 'Z':
                return orientation.firstAngle;
            case 'Y':
                return orientation.thirdAngle;
            case 'X':
                return orientation.secondAngle;
        }
        return 0;
    }
    //Tile
    int getInX() {
        return innerTile.x;
    }

    int getInY() {
        return innerTile.y;
    }

    private double[] getDist(Tile start, Tile end, int dir) {
        double forward, left;
        /* //Note: Code to apply for any given angle. Not sure if it works, so it's commented. Let's tests.
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
        forward = finalVerticalAdjust - startVerticalAdjust;
        left = finalHorizontalAdjust - startHorizontalAdjust;
        return new double[]{tilesToInch(forward), tilesToInch(left)};
        }
private void moveToCenter(double left, double right) {
        double P = 0.02;
        double error =  right - left;
        double speed;
        double minSpeed = 0.01;
        double maxSpeed = 0.03;
        speed = Range.clip(P * error, minSpeed, maxSpeed);
        telemetry.addData("left", left);
        telemetry.addData("right", right);
        telemetry.update();

}
        */


}
