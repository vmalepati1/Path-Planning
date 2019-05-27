package org.waltonrobotics.geometry.clipping;

import org.waltonrobotics.geometry.Vector2f;

public class Epsilon {

    private double eps;

    public Epsilon() {
        this.eps = 0.0000000001;
    }

    public Epsilon(double eps) {
        this.eps = eps;
    }

    public boolean pointAboveOrOnLine(Vector2f pt, Vector2f left, Vector2f right) {
        double ax = left.getX();
        double ay = left.getY();
        double bx = right.getX();
        double by = right.getY();
        double cx = pt.getX();
        double cy = pt.getY();
        return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax) >= -eps;
    }

    private boolean pointsSameX(Vector2f p1, Vector2f p2) {
        return Math.abs(p1.getX() - p2.getX()) < eps;
    }

    private boolean pointsSameY(Vector2f p1, Vector2f p2) {
        return Math.abs(p1.getY() - p2.getY()) < eps;
    }

    private boolean pointsSame(Vector2f p1, Vector2f p2) {
        return pointsSameX(p1, p2) && pointsSameY(p1, p2);
    }

    public int pointsCompare(Vector2f p1, Vector2f p2) {
        if (pointsSameX(p1, p2)) {
            return pointsSameY(p1, p2) ? 0 : (p1.getY() < p2.getY() ? -1 : 1);
        }

        return p1.getX() < p2.getX() ? -1 : 1;
    }

}
