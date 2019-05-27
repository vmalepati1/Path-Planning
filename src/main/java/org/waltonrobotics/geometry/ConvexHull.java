package org.waltonrobotics.geometry;

import java.io.Serializable;
import java.util.*;

/**
 * Implementation of a convex hull.
 */
public class ConvexHull implements Serializable {

    private static Comparator<Vector2f> rightBottomComparator = (v1, v2) -> {
        int compareX = v1.compareX(v2);
        int compareY = v1.compareY(v2);

        if (compareX == 0) {
            return compareY;
        }

        return compareX;
    };
    private LinkedHashSet<Vector2f> allPoints;
    private List<Vector2f> convexPoints;
    private List<LineSegment> edges;
    private Rectangle boundingBox;

    /**
     * Constructs a new convex hull.
     */
    public ConvexHull() {
        allPoints = new LinkedHashSet<>();
        convexPoints = new ArrayList<>();
        edges = new ArrayList<>();
        boundingBox = new Rectangle();
    }

    /**
     * Checks if p0->p1->p2 form a counter-clockwise turn like shown below:
     * p2
     * p1
     * p0
     *
     * @param p0: First point
     * @param p1: Second point
     * @param p2: Third point
     * @return > 0 for counter-clockwise turn
     * = 0 for collinear
     * < 0 for clockwise turn
     */
    private static double pointsCCWTurnTest(Vector2f p0, Vector2f p1, Vector2f p2) {
        return ((p1.getX() - p0.getX()) * (p2.getY() - p0.getY())
                - (p2.getX() - p0.getX()) * (p1.getY() - p0.getY()));
    }

    /**
     * Computes convex hull of given points using algorithm found in this paper:
     * A.M. Andrew, "Another Efficient  Algorithm for Convex Hulls in Two Dimensions", Info.  Proc. Letters 9, 216-219 (1979)
     *
     * @param points: Points to perform convex hull on
     * @return List of convex hull points
     */
    private List<Vector2f> ChainConvexHull(LinkedHashSet<Vector2f> points) {
        // Holds points sorted in increasing x order and then y order
        List<Vector2f> sortedPoints = new ArrayList<>(points);
        // Stack of result points
        Stack<Vector2f> resultStack = new Stack<>();

        // Sort given points by increasing x-coordinate and then y-coordinate
        sortedPoints.sort(rightBottomComparator);

        // Total n points
        int totalPoints = sortedPoints.size();
        // Index of points with min x first and min y second
        int bottomLeftIndex = 0;
        // Index of points with min x first and max y second
        int topLeftIndex = 0;
        // Index of points with max x first and min y second
        int bottomRightIndex = 0;
        // Index of points with max x first and max y second
        int topRightIndex = totalPoints - 1;

        // Minimum x value within all points
        double minX = sortedPoints.get(bottomLeftIndex).getX();
        // Minimum y value within all points
        double minY = sortedPoints.get(bottomLeftIndex).getY();
        // Maximum x value within all points
        double maxX = sortedPoints.get(bottomRightIndex).getX();
        // Maximum y value within all points
        double maxY = sortedPoints.get(topRightIndex).getY();

        boundingBox = new Rectangle(minX, maxX, minY, maxY);

        // Find our top-left index
        int i = 0;

        for (i = 1; i < totalPoints; i++) {
            if (sortedPoints.get(i).getX() != minX) {
                break;
            }
        }

        topLeftIndex = i - 1;

        Vector2f bottomLeftPoint = sortedPoints.get(bottomLeftIndex);
        Vector2f topLeftPoint = sortedPoints.get(topLeftIndex);

        // Degenerate case: all x-coords == xmin
        if (topLeftIndex == totalPoints - 1) {
            resultStack.push(bottomLeftPoint);

            // Check for non-trivial segment
            if (topLeftPoint.getY() != bottomLeftPoint.getY()) {
                resultStack.push(topLeftPoint);
            }

            // Add polygon endpoint
            resultStack.push(bottomLeftPoint);

            return resultStack;
        }

        for (i = totalPoints - 2; i >= 0; i--) {
            if (sortedPoints.get(i).getX() != maxX) {
                break;
            }
        }

        // Find our bottom-right index
        bottomRightIndex = i + 1;

        Vector2f bottomRightPoint = sortedPoints.get(bottomRightIndex);
        Vector2f topRightPoint = sortedPoints.get(topRightIndex);

        resultStack.push(bottomLeftPoint);

        i = topLeftIndex;

        // Compute lower hull
        while (++i <= bottomRightIndex) {
            Vector2f currentPoint = sortedPoints.get(i);

            // The lower line joins points[minmin] with points[maxmin]
            if (pointsCCWTurnTest(bottomLeftPoint, bottomRightPoint, currentPoint) >= 0
                    && i < bottomRightIndex)
                continue;

            while (resultStack.size() - 1 > 0) {
                // Test if  P[i] is left of the line at the stack top
                Vector2f top = resultStack.pop();

                if (pointsCCWTurnTest(resultStack.peek(), top, currentPoint) > 0) {
                    resultStack.push(top);
                    break;
                }
            }

            resultStack.push(currentPoint);
        }

        if (topRightIndex != bottomRightIndex)
            resultStack.push(topRightPoint);

        int bottom = resultStack.size() - 1;

        i = bottomRightIndex;

        // Compute upper hull
        while (--i >= topLeftIndex) {
            Vector2f currentPoint = sortedPoints.get(i);

            // The upper line joins points[maxmax]  with points[minmax]
            if (pointsCCWTurnTest(topRightPoint, topLeftPoint, currentPoint) >= 0 && i > topLeftIndex)
                continue;

            while (resultStack.size() - 1 > bottom) {
                Vector2f top = resultStack.pop();

                if (pointsCCWTurnTest(resultStack.peek(), top, currentPoint) > 0) {
                    resultStack.push(top);
                    break;
                }
            }

            resultStack.push(currentPoint);
        }

        if (topLeftIndex != bottomLeftIndex) {
            // Push joining endpoint onto stack
            resultStack.push(bottomLeftPoint);
        }

        return new ArrayList<>(resultStack);
    }

    /**
     * @return All points added before performing convex hull
     */
    public LinkedHashSet<Vector2f> getAllPoints() {
        return allPoints;
    }

    /**
     * @return Convex hull points
     */
    public List<Vector2f> getConvexPoints() {
        return convexPoints;
    }


    /**
     * @return Edges of convex hull
     */
    public List<LineSegment> getEdges() {
        return edges;
    }

    /**
     * Starts allowing adding points to convex hull.
     */
    public void begin() {
        allPoints.clear();
        convexPoints.clear();
        edges.clear();
    }

    /**
     * Adds a point to convex hull.
     *
     * @param point: Point to add
     */
    public void addPoint(Vector2f point) {
        allPoints.add(point);
    }

    /**
     * Removes point from convex hull.
     *
     * @param point: Point to remove
     */
    public void removePoint(Vector2f point) {
        allPoints.remove(point);
    }

    /**
     * Computes convex hull points and edges.
     */
    public void end() {
        convexPoints = ChainConvexHull(allPoints);

        for (int i = 1; i < convexPoints.size(); i++) {
            Vector2f currentPoint = convexPoints.get(i);
            Vector2f lastPoint = convexPoints.get(i - 1);

            edges.add(new LineSegment(lastPoint, currentPoint));
        }

        convexPoints.remove(convexPoints.size() - 1);
    }

    /**
     * Calculates area of convex hull using Shoelace formula.
     *
     * @return Area of convex hull
     */
    public double getArea() {
        // Initialize area
        double area = 0.0;

        // Calculate value of Shoelace formula

        int n = convexPoints.size();

        int j = n - 1;
        for (int i = 0; i < n; i++) {
            area += (convexPoints.get(j).getX() + convexPoints.get(i).getX()) * (convexPoints.get(j).getY() - convexPoints.get(i).getY());

            // J is previous vertex to i
            j = i;
        }

        // Return absolute value
        return Math.abs(area / 2.0);
    }

}
