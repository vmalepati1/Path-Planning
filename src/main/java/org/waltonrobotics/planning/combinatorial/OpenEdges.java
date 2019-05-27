package org.waltonrobotics.planning.combinatorial;

import org.waltonrobotics.geometry.LineSegment;
import org.waltonrobotics.geometry.Vector2f;

import java.util.ArrayList;
import java.util.List;

public class OpenEdges {

    private List<LineSegment> openEdges;

    public OpenEdges() {
        openEdges = new ArrayList<>();
    }

    public List<LineSegment> getOpenEdges() {
        return openEdges;
    }

    public void insert(Vector2f halfLineOrigin, Vector2f w, LineSegment edge) {
        openEdges.add(index(halfLineOrigin, w, edge), edge);
    }

    public void delete(Vector2f halfLineOrigin, Vector2f w, LineSegment edge) {
        int index = index(halfLineOrigin, w, edge) - 1;

        if (index < 0 || index >= openEdges.size()) return;

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
            } else {
                low = mid + 1;
            }
        }

        return low;
    }

    private boolean lessThan(Vector2f halfLineOrigin, Vector2f w, LineSegment edge1, LineSegment edge2) {
        if (edge1.equals(edge2)) {
            return false;
        }

        LineSegment scanLine = new LineSegment(halfLineOrigin, w);

        if (!VisibilityGraph.edgeIntersect(halfLineOrigin, w, edge2)) {
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
            double maxDistanceFromHalfLineOrigin1 = Math.max(edge1.getPoint1().distanceTo(halfLineOrigin), edge1.getPoint2().distanceTo(halfLineOrigin));
            double maxDistanceFromHalfLineOrigin2 = Math.max(edge2.getPoint1().distanceTo(halfLineOrigin), edge2.getPoint2().distanceTo(halfLineOrigin));

            return maxDistanceFromHalfLineOrigin1 < maxDistanceFromHalfLineOrigin2;
        }

        return false;
    }

}
