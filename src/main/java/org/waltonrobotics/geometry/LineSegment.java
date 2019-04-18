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

    public Vector2f getAdjacent(Vector2f point) {
        if (!point1.equals(point) && !point2.equals(point)) return null;

        return (point1.equals(point) ? point2 : point1);
    }

    private void calculateSlope() {
        slope = (point2.getY() - point1.getY()) / (point2.getX() - point1.getX());
    }

    @Override
    public String toString() {
        return "Line: {" + point1 + " -> " + point2 + "}";
    }

    @Override
    public boolean equals(Object object) {
        if (this == object) return true;
        if (object == null) return false;
        if (!(object instanceof LineSegment)) return false;
        LineSegment lineSegment = (LineSegment) object;
        return lineSegment.getPoint1() == this.getPoint1() && lineSegment.getPoint2() == this.getPoint2();
    }

    @Override
    public int hashCode() {
        return point1.hashCode() + point2.hashCode();
    }

    public Vector2f getIntersectionPoint(LineSegment other) {
        if (other.getPoint1().equals(point1) || other.getPoint2().equals(point1)) return point1;
        if (other.getPoint1().equals(point2) || other.getPoint2().equals(point2)) return point2;

        double pSlope = (point1.getY() - point2.getY()) / (point1.getX() - point2.getX());
        double oSlope = (other.getPoint1().getY() - other.getPoint2().getY()) / (other.getPoint1().getX() - other.getPoint2().getX());

        if (other.getPoint1().getX() == other.getPoint2().getX()) {
            if (point1.getX() == point2.getX()) {
                return null;
            }

            double intersectX = other.getPoint1().getX();
            double intersectY = pSlope * (intersectX - point1.getX()) + point1.getY();

            return new Vector2f(intersectX, intersectY);
        }

        if (point1.getX() == point2.getX()) {
            double intersectX = point1.getX();
            double intersectY = oSlope * (intersectX - other.getPoint1().getX()) + other.getPoint1().getY();

            return new Vector2f(intersectX, intersectY);
        }

        if (pSlope == oSlope) {
            return null;
        }

        double intersectX = (oSlope * other.getPoint1().getX() - pSlope * point1.getX() + point1.getY() - other.getPoint1().getY()) / (oSlope - pSlope);
        double intersectY = oSlope * (intersectX - other.getPoint1().getX()) + other.getPoint1().getY();
        return new Vector2f(intersectX, intersectY);
    }

    public double getPointEdgeDistance(LineSegment edge) {
        Vector2f ip = getIntersectionPoint(edge);

        if (ip != null) {
            return point1.distanceTo(ip);
        }

        return 0;
    }

}
