package org.waltonrobotics.geometry;

import java.io.Serializable;

public class LineSegment implements Serializable {

    private Vector2f point1;
    private Vector2f point2;
    private double slope;

    public LineSegment(Vector2f point1, Vector2f point2) {
        this.point1 = point1;
        this.point2 = point2;

        calculateSlope();
    }

    public Vector2f getPoint1() {
        return point1;
    }

    public void setPoint1(Vector2f point1) {
        this.point1 = point1;
        calculateSlope();
    }

    public Vector2f getPoint2() {
        return point2;
    }

    public void setPoint2(Vector2f point2) {
        this.point2 = point2;
        calculateSlope();
    }

    public double getSlope() {
        return slope;
    }

    private void calculateSlope() {
        slope = (point2.getY() - point1.getY()) / (point2.getX() - point1.getX());
    }

}
