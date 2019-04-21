package org.waltonrobotics.planning.combinatorial;

import org.waltonrobotics.geometry.ConvexHull;
import org.waltonrobotics.geometry.LineSegment;
import org.waltonrobotics.geometry.Pose;
import org.waltonrobotics.geometry.Vector2f;

import java.util.*;

public class VisibilityGraph {

    private static class CCWHalfLineComparator implements Comparator<Vector2f> {

        private Vector2f halfLineOrigin;

        private CCWHalfLineComparator(Vector2f halfLineOrigin) {
            this.halfLineOrigin = halfLineOrigin;
        }

        private double getAngleFromHalfLineToPoint(Vector2f point) {
            double dx = point.getX() - halfLineOrigin.getX();
            double dy = point.getY() - halfLineOrigin.getY();

            if (dx == 0) {
                if (dy < 0) {
                    return Math.PI * 3 / 2;
                }

                return Math.PI / 2;
            }

            if (dy == 0) {
                if (dx < 0) {
                    return Math.PI;
                }

                return 0;
            }

            if (dx < 0) {
                return Math.PI + Math.atan(dy / dx);
            }

            if (dy < 0) {
                return 2 * Math.PI + Math.atan(dy / dx);
            }

            return Math.atan(dy / dx);
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
                double halfLineOriginToV1 = halfLineOrigin.distanceTo(v1);
                double halfLineOriginToV2 = halfLineOrigin.distanceTo(v2);

                if (halfLineOriginToV1 > halfLineOriginToV2) {
                    return 1;
                } else if (halfLineOriginToV1 < halfLineOriginToV2) {
                    return -1;
                }
            }

            return 0;
        }

    }

    public List<Vector2f> getVertices() {
        return vertices;
    }

    public List<LineSegment> getEdges() {
        return edges;
    }

    public List<ConvexHull> getObstacles() {
        return obstacles;
    }

    public double getRobotOrientationDegrees() {
        return robotOrientationDegrees;
    }

    public Map<Pose, List<Pose>> getGraphAdjacencyList() {
        return graphAdjacencyList;
    }

    private List<Vector2f> vertices;
    private List<LineSegment> edges;
    private List<ConvexHull> obstacles;
    private double robotOrientationDegrees;
    private Map<Pose, List<Pose>> graphAdjacencyList;

    public VisibilityGraph(List<Vector2f> vertices, List<LineSegment> edges, List<ConvexHull> obstacles, double robotOrientationDegrees) {
        this.vertices = vertices;
        this.edges = edges;
        this.obstacles = obstacles;
        this.robotOrientationDegrees = robotOrientationDegrees;

        calculateVisibilityGraph();
    }

    private void calculateVisibilityGraph() {
        graphAdjacencyList = new HashMap<>();

        for (Vector2f v : vertices) {
            Pose source = new Pose(v.getX(), v.getY(), robotOrientationDegrees);

            List<Pose> sourceAdjacent = new ArrayList<>();

            for (Vector2f visibleVertex : getVisibleVertices(v, vertices, edges, obstacles)) {
                Pose destination = new Pose(visibleVertex.getX(), visibleVertex.getY(), robotOrientationDegrees);
                List<Pose> destinationAdjacent = graphAdjacencyList.get(source);

                if (destinationAdjacent == null) {
                    destinationAdjacent = new ArrayList<>();
                }

                sourceAdjacent.add(destination);
                destinationAdjacent.add(source);

                graphAdjacencyList.put(source, sourceAdjacent);
                graphAdjacencyList.put(destination, destinationAdjacent);
            }
        }
    }

    private List<Vector2f> getVisibleVertices(Vector2f point, List<Vector2f> vertices, List<LineSegment> edges, List<ConvexHull> obstacles) {
        List<Vector2f> sortedVertices = new ArrayList<>(vertices);
        List<Vector2f> visibleVertices = new ArrayList<>();

        sortedVertices.sort(new CCWHalfLineComparator(point));

        OpenEdges openEdges = new OpenEdges();

        Vector2f pointInf = new Vector2f(Double.POSITIVE_INFINITY, point.getY());

        for (LineSegment e : edges) {
            if (e.getPoint1().equals(point) || e.getPoint2().equals(point)) continue;

            if (edgeIntersect(point, pointInf, e)) {
                if (onSegment(point, e.getPoint1(), pointInf)) continue;
                if (onSegment(point, e.getPoint2(), pointInf)) continue;
                openEdges.insert(point, pointInf, e);
            }
        }

        Vector2f previous = null;
        boolean previousVisible = false;

        for (Vector2f p : sortedVertices) {
            if (p.equals(point)) continue;

            if (!openEdges.getOpenEdges().isEmpty()) {
                for (LineSegment e : edges) {
                    if (e.getPoint1().equals(p) || e.getPoint2().equals(p)) {
                        if (isCCW(point, p, e.getAdjacent(p)) == -1) {
                            openEdges.delete(point, p, e);
                        }
                    }
                }
            }

            boolean isVisible = false;

            if (previous == null || isCCW(point, previous, p) != 0 || !onSegment(point, previous, p)) {
                if (openEdges.getOpenEdges().size() == 0) {
                    isVisible = true;
                } else if (!edgeIntersect(point, p, openEdges.smallest())) {
                    isVisible = true;
                }
            } else if (!previousVisible) {
                isVisible = false;
            } else {
                isVisible = true;

                for (LineSegment e : openEdges.getOpenEdges()) {
                    if (!e.getPoint1().equals(previous) && !e.getPoint2().equals(previous) && edgeIntersect(previous, p, e)) {
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

                if (!e.getPoint1().equals(point) && !e.getPoint2().equals(point) && e.getAdjacent(p) != null) {
                    if (isCCW(point, p, e.getAdjacent(p)) == 1) {
                        openEdges.insert(point, p, e);
                    }
                }
            }

            if (isVisible && !adjacentPoints.contains(p)) {
                isVisible = !edgeInPolygon(point, p, obstacles);
            }

            if (isVisible) {
                visibleVertices.add(p);
            }

            previous = p;
            previousVisible = isVisible;
        }

        return visibleVertices;
    }

    public static boolean edgeIntersect(Vector2f p1, Vector2f q1, LineSegment edge) {
        Vector2f p2 = edge.getPoint1();
        Vector2f q2 = edge.getPoint2();

        int o1 = isCCW(p1, q1, p2);
        int o2 = isCCW(p1, q1, q2);
        int o3 = isCCW(p2, q2, p1);
        int o4 = isCCW(p2, q2, q1);

        if (o1 != o2 && o3 != o4) {
            return true;
        }

        if (o1 == 0 && onSegment(p1, p2, q1)) {
            return true;
        }

        if (o2 == 0 && onSegment(p1, q2, q1)) {
            return true;
        }

        if (o3 == 0 && onSegment(p2, p1, q2)) {
            return true;
        }

        return o4 == 0 && onSegment(p2, q1, q2);
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

            boolean edgeP1Collinear = (isCCW(midPoint, e.getPoint1(), p2) == 0);
            boolean edgeP2Collinear = (isCCW(midPoint, e.getPoint2(), p2) == 0);

            if (edgeP1Collinear && edgeP2Collinear) continue;
            if (edgeP1Collinear || edgeP2Collinear) {
                Vector2f collinearPoint = edgeP1Collinear ? e.getPoint1() : e.getPoint2();

                if (e.getAdjacent(collinearPoint).getY() > midPoint.getY()) {
                    intersectCount++;
                }
            } else if (edgeIntersect(midPoint, p2, e)) {
                intersectCount++;
            }
        }

        return intersectCount % 2 != 0;
    }

    private static int isCCW(Vector2f a, Vector2f b, Vector2f c) {
        double area = ((b.getX() - a.getX()) * (c.getY() - a.getY()) - (b.getY() - a.getY()) * (c.getX() - a.getX()));

        if (area > 0) return 1;
        if (area < 0) return -1;
        return 0;
    }

    private static boolean onSegment(Vector2f p, Vector2f q, Vector2f r) {
        if (q.getX() <= Math.max(p.getX(), r.getX()) && (q.getX() >= Math.min(p.getX(), r.getX()))) {
            return q.getY() <= Math.max(p.getY(), r.getY()) && (q.getY() >= Math.min(p.getY(), r.getY()));
        }

        return false;
    }

}
