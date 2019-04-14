package org.waltonrobotics.planning.combinatorial;

import org.waltonrobotics.geometry.ConvexHull;
import org.waltonrobotics.geometry.LineSegment;
import org.waltonrobotics.geometry.Vector2f;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public class VisibilityGraph {

    private static class CCWHalfLineComparator implements Comparator<Vector2f> {

        private Vector2f halfLineOrigin;

        public CCWHalfLineComparator(Vector2f halfLineOrigin) {
            this.halfLineOrigin = halfLineOrigin;
        }

        private double getAngleFromHalfLineToPoint(Vector2f point) {
            double angle = Math.toDegrees(Math.atan2(point.getY() - halfLineOrigin.getY(), point.getX() - halfLineOrigin.getX()));
            return (angle %= 360) < 0 ? angle + 360 : angle;
        }

        @Override
        public int compare(Vector2f v1, Vector2f v2) {
            double v1CCWAngle = getAngleFromHalfLineToPoint(v1);
            double v2CCWAngle = getAngleFromHalfLineToPoint(v2);

            if (v1CCWAngle > v2CCWAngle) {
                return 1;
            } else if (v1CCWAngle < v2CCWAngle) {
                return -1;
            } else {
                double v1ToHalfLineOrigin = v1.distanceTo(halfLineOrigin);
                double v2ToHalfLineOrigin = v2.distanceTo(halfLineOrigin);

                if (v1ToHalfLineOrigin > v2ToHalfLineOrigin) {
                    return 1;
                } else if (v1ToHalfLineOrigin < v2ToHalfLineOrigin) {
                    return -1;
                }
            }

            return 0;
        }

    }

    public static List<Vector2f> calculateVisibilityGraph(List<Vector2f> vertices, List<LineSegment> edges, List<ConvexHull> obstacles) {
        List<Vector2f> visibilityGraph = new ArrayList<>();

        for (Vector2f v : vertices) {
            visibilityGraph.addAll(getVisibleVertices(v, vertices, edges, obstacles));
        }

        return visibilityGraph;
    }

    private static List<Vector2f> getVisibleVertices(Vector2f point, List<Vector2f> vertices, List<LineSegment> edges, List<ConvexHull> obstacles) {
        List<Vector2f> sortedVertices = new ArrayList<>(vertices);
        List<Vector2f> visibleVertices = new ArrayList<>();

        sortedVertices.sort(new CCWHalfLineComparator(point));

        System.out.println(sortedVertices);

        OpenEdges openEdges = new OpenEdges();

        Vector2f pointInf = new Vector2f(Double.POSITIVE_INFINITY, point.getY());

        LineSegment scanLine = new LineSegment(point, pointInf);

        //System.out.println(edges);

        for (LineSegment e : edges) {
            if (e.getPoint1().equals(point) || e.getPoint2().equals(point)) continue;

            if (scanLine.doesIntersect(e)) {
                if (onSegment(point, e.getPoint1(), pointInf)) continue;
                if (onSegment(point, e.getPoint2(), pointInf)) continue;
                openEdges.insert(point, pointInf, e);
            }
        }

        Vector2f previous = null;
        boolean previousVisible = false;

        for (Vector2f p : sortedVertices) {
            if (p.equals(point)) continue;

            scanLine = new LineSegment(point, p);

            if (!openEdges.getOpenEdges().isEmpty()) {
                for (LineSegment e : edges) {
                    if (e.getPoint1().equals(p) || e.getPoint2().equals(p)) {
                        if (isCCW(scanLine, e.getAdjacent(p)) == -1) {
                            openEdges.delete(point, p, e);
                        }
                    }
                }
            }

            boolean isVisible = false;

            if (previous == null || isCCW(new LineSegment(point, previous), p) != 0 || !onSegment(point, previous, p)) {
                if (openEdges.getOpenEdges().size() == 0) {
                    isVisible = true;
                } else if (!scanLine.doesIntersect(openEdges.smallest())) {
                    isVisible = true;
                }
            } else if (!previousVisible) {
                isVisible = false;
            } else {
                isVisible = true;

                for (LineSegment e : openEdges.getOpenEdges()) {
                    LineSegment previousToCurrent = new LineSegment(previous, p);
                    if (!e.getPoint1().equals(previous) && !e.getPoint2().equals(previous) && previousToCurrent.doesIntersect(e)) {
                        isVisible = false;
                        break;
                    }

                    if (isVisible && edgeInPolygon(previous, p, obstacles)) {
                        isVisible = false;
                    }
                }
            }

            List<Vector2f> adjacentPoints = new ArrayList<>();

            for (LineSegment e : edges) {
                if (e.getPoint1().equals(point) || e.getPoint2().equals(point)) {
                    adjacentPoints.add(e.getAdjacent(point));
                }
            }

            if (isVisible && !adjacentPoints.contains(p)) {
                isVisible = !edgeInPolygon(point, p, obstacles);
            }

            if (isVisible) {
                visibleVertices.add(p);
                System.out.println(point + " -> " + p);
            }

            for (LineSegment e : edges) {
                if (!e.getPoint1().equals(point) && !e.getPoint2().equals(point) && e.getAdjacent(p) != null) {
                    if (isCCW(scanLine, e.getAdjacent(p)) == 1) {
                        openEdges.insert(point, p, e);
                    }
                }
            }

            previous = p;
            previousVisible = isVisible;
        }

        return visibleVertices;
    }

    private static boolean edgeInPolygon(Vector2f p1, Vector2f p2, List<ConvexHull> obstacles) {
        if (p1.getPolygonID() != p2.getPolygonID()) {
            return false;
        }
        
        if (p1.getPolygonID() == -1 || p2.getPolygonID() == -1) {
            return false;
        }
        
        Vector2f midPoint = new Vector2f((p1.getX() + p2.getX()) / 2, (p1.getY() + p2.getY()) / 2);
        
        return polygonCrossing(midPoint, obstacles.get(p1.getPolygonID()));
    }

    private static boolean polygonCrossing(Vector2f midPoint, ConvexHull convexHull) {
        Vector2f p2 = new Vector2f(Double.POSITIVE_INFINITY, midPoint.getY());

        int intersectCount = 0;

        for (LineSegment e : convexHull.getEdges()) {
            if (midPoint.getY() < e.getPoint1().getY() && midPoint.getY() < e.getPoint2().getY()) continue;
            if (midPoint.getY() > e.getPoint1().getY() && midPoint.getY() > e.getPoint2().getY()) continue;
            if (midPoint.getX() > e.getPoint1().getX() && midPoint.getX() > e.getPoint2().getX()) continue;

            boolean edgeP1Collinear = (isCCW(new LineSegment(midPoint, e.getPoint1()), p2) == 0);
            boolean edgeP2Collinear = (isCCW(new LineSegment(midPoint, e.getPoint2()), p2) == 0);

            LineSegment scanLine = new LineSegment(midPoint, p2);

            if (edgeP1Collinear && edgeP2Collinear) continue;
            if (edgeP1Collinear || edgeP2Collinear) {
                Vector2f collinearPoint = edgeP1Collinear ? e.getPoint1() : e.getPoint2();

                if (e.getAdjacent(collinearPoint).getY() > midPoint.getY()) {
                    intersectCount++;
                }
            } else if (scanLine.doesIntersect(e)) {
                intersectCount++;
            }
        }

        return intersectCount % 2 != 0;
    }

    public static int isCCW(LineSegment scanLine, Vector2f adjacentPoint) {
        double area = ((scanLine.getPoint2().getX() - scanLine.getPoint1().getX()) * (adjacentPoint.getY() - scanLine.getPoint1().getY()) -
                (scanLine.getPoint2().getY() - scanLine.getPoint1().getY()) * (adjacentPoint.getX() - scanLine.getPoint1().getX()));

        if (area > 0) return 1;
        if (area < 0) return -1;
        return 0;
    }

    public static boolean onSegment(Vector2f p, Vector2f q, Vector2f r) {
        if (q.getX() <= Math.max(p.getX(), r.getX()) && (q.getX() >= Math.min(p.getX(), r.getX()))) {
            if (q.getY() <= Math.max(p.getY(), r.getY()) && (q.getY() >= Math.min(p.getY(), r.getY()))) {
                return true;
            }
        }

        return false;
    }

}
