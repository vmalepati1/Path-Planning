package org.waltonrobotics.planning.combinatorial;

import org.waltonrobotics.geometry.LineSegment;
import org.waltonrobotics.geometry.Vector2f;

import java.util.ArrayList;
import java.util.List;

public class OpenEdges {

    public List<LineSegment> getOpenEdges() {
        return openEdges;
    }

    private List<LineSegment> openEdges;

    public OpenEdges() {
        openEdges = new ArrayList<>();
    }

    public void insert(Vector2f halfLineOrigin, Vector2f w, LineSegment edge) {
        openEdges.add(index(halfLineOrigin, w, edge), edge);
    }

    public void delete(Vector2f halfLineOrigin, Vector2f w, LineSegment edge) {
        int index = index(halfLineOrigin, w, edge) - 1;
        if (openEdges.get(index).equals(edge)) {
            openEdges.remove(index);
        }
    }

    public LineSegment smallest() {
        return openEdges.get(0);
    }

    private int index(Vector2f halfLineOrigin, Vector2f w, LineSegment edge) {
        int low = 0;
        int high = openEdges.size();

        while (low < high) {
            int mid = (low + high) / 2;

            if (lessThan(halfLineOrigin, w, edge, openEdges.get(mid))) {
                high = mid;
            } else{
                low = mid + 1;
            }
        }

        return low;
    }

    private double angleBetween(Vector2f a, Vector2f b, Vector2f c) {
        double first = Math.pow((c.getX() - b.getX()), 2) + Math.pow(c.getY() - b.getY(), 2);
        double second = Math.pow((c.getX() - a.getX()), 2) + Math.pow(c.getY() - a.getY(), 2);
        double third = Math.pow((b.getX() - a.getX()), 2) + Math.pow(b.getY() - a.getY(), 2);
        double cosValue = (first + third - second) / (2 * Math.sqrt(first) * Math.sqrt(third));
        return Math.acos(cosValue);

    }

    private boolean lessThan(Vector2f halfLineOrigin, Vector2f w, LineSegment edge1, LineSegment edge2) {
        if (edge1.equals(edge2)) {
            return false;
        }

        LineSegment scanLine = new LineSegment(halfLineOrigin, w);

        if (!scanLine.doesIntersect(edge2)) {
            return true;
        }

        double edgeDist1 = scanLine.getPointEdgeDistance(edge1);
        double edgeDist2 = scanLine.getPointEdgeDistance(edge2);

        if (edgeDist1 > edgeDist2) {
            return false;
        }

        if (edgeDist1 < edgeDist2) {
            return true;
        }

        if (edgeDist1 == edgeDist2) {
            Vector2f samePoint;

            if (edge2.getPoint1().equals(edge1.getPoint1()) || edge2.getPoint2().equals(edge1.getPoint1())) {
                samePoint = edge1.getPoint1();
            } else {
                samePoint = edge1.getPoint2();
            }

            double angleEdge1 = angleBetween(halfLineOrigin, w, edge1.getAdjacent(samePoint));
            double angleEdge2 = angleBetween(halfLineOrigin, w, edge2.getAdjacent(samePoint));

            if (angleEdge1 < angleEdge2) {
                return true;
            }
        }

        return false;
    }

}
