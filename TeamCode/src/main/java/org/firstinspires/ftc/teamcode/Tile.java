package org.firstinspires.ftc.teamcode;

import android.graphics.Point;

public class Tile {
    private double x, y;
    Point innerTile;


    Tile(double x1, double y1, int x2, int y2) {
        innerTile = new Point(x2, y2);
        setLocation(x1,y1);
        setRealPos();
    }

    Tile(double x, double y) {
        setLocation(x,y);
        innerTile = new Point(2, 2);
        setRealPos();
    }



    Tile(Tile tile) {
        setLocation(tile);
    }

    double getX() {
        return x;
    }

    double getY() {
        return y;
    }

    int getInX() {
        return innerTile.x; }

    int getInY() {
        return innerTile.y;
    }

    public void setLocation(double x, double y) {
        this.x = x;
        this.y = y;
    }

    void setLocation(Tile tile) {
        this.x = tile.getX();
        this.y = tile.getY();
    }

    void add(double x, double y) {
        this.x += x;
        this.y += y;
    }

    private void setRealPos() {
        if(innerTile.x > 2) {
            setLocation(x + 1/3d, y);
        } else if(innerTile.x < 2) {
            setLocation( - 1/3d,y);
        }

        if (innerTile.y> 2) {
            setLocation(x,y + 1/3d);
        } else if (innerTile.y < 2) {
            setLocation(x,y - 1/3d);
        }
    }

    Tile flip() {
        double x;
        int x2;
        x = this.x * -1 + 7;
        x2 = this.innerTile.x * -1 + 4;
        return new Tile(x, y, x2, this.innerTile.y);
    }


}