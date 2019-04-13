package org.waltonrobotics.geometry;

import java.util.ArrayList;
import java.util.List;

/**
 * GJK collision algorithm implementation used for finding intersections between polygons.
 * Algorithm described in detail here: https://en.wikipedia.org/wiki/Gilbert–Johnson–Keerthi_distance_algorithm
 */
public class GJKCollision {

    /**
     * Finds the farthest point in a polygon in a specified direction
     *
     * @param polygon:   List of points in polygon
     * @param direction: Direction for farthest point
     * @return Farthest point in polygon in given direction
     */
    private static Vector2f farthestPointInDirection(List<Vector2f> polygon, Vector2f direction) {
        Vector2f farthestPoint = polygon.get(0);

        double farthestDistance = farthestPoint.dot(direction);

        double searchingDistance = 0;

        for (Vector2f v : polygon) {
            searchingDistance = v.dot(direction);

            if (searchingDistance > farthestDistance) {
                farthestDistance = searchingDistance;
                farthestPoint = v;
            }
        }

        // Return the point within the polygon with the highest dot product with the direction
        return farthestPoint;
    }

    /**
     * The support function builds the simplex.
     *
     * @param shape1:    First polygon
     * @param shape2:    Second polygon
     * @param direction: Specified direction to perform Minkowski Difference on
     * @return Point in Minkowski space on the edge of the Minkowski Difference
     */
    private static Vector2f support(List<Vector2f> shape1, List<Vector2f> shape2, Vector2f direction) {
        // Direction is a vector direction (doesn't have to be normalized)
        // Get points on the edge of the shapes in opposite directions
        Vector2f p1 = farthestPointInDirection(shape1, direction);
        Vector2f p2 = farthestPointInDirection(shape2, direction.negative());

        // Perform the Minkowski Difference
        Vector2f p3 = p1.subtract(p2);

        // P3 is now a point in Minkowski space on the edge of the Minkowski Difference
        return p3;
    }

    /**
     * @param simplex:   Current built simplex
     * @param direction: Direction of search
     * @return True if simplex contains origin, false otherwise
     */
    private static boolean containsOrigin(List<Vector2f> simplex, Vector2f direction) {
        // Get the last point added to the simplex
        Vector2f a = simplex.get(simplex.size() - 1);

        // Compute AO (same thing as -A)
        Vector2f ao = a.negative();

        // Triangle case
        if (simplex.size() == 3) {
            Vector2f b = simplex.get(1);
            Vector2f c = simplex.get(0);

            // Compute the edges
            Vector2f ab = b.subtract(a);
            Vector2f ac = c.subtract(a);

            // Compute the normals
            Vector2f abPerp = ac.tripleProduct(ab, ab);
            Vector2f acPerp = ab.tripleProduct(ac, ac);

            // Is the origin in R4
            if (abPerp.dot(ao) > 0) {
                // Remove point c
                simplex.remove(c);

                // Set the new search direction to abPerp
                direction.set(abPerp);
            } else {
                // Is the origin in R3
                if (acPerp.dot(ao) > 0) {
                    // Remove point b
                    simplex.remove(b);

                    // Set the new search direction to acPerp
                    direction.set(acPerp);
                } else {
                    // Otherwise origin is in R5 so we can return true
                    return true;
                }
            }
        } else {
            // Line segment case
            Vector2f b = simplex.get(0);

            // Compute AB
            Vector2f ab = b.subtract(a);

            // Get the perpendicular line to AB in the direction of the origin
            Vector2f abPerp = ab.tripleProduct(ao, ab);

            // Set the new search direction to abPerp
            direction.set(abPerp);
        }

        return false;
    }

    /**
     * @param shape1: First polygon
     * @param shape2: Second polygon
     * @return True if polygons intersect, false otherwise
     */
    public static boolean polygonsIntersect(List<Vector2f> shape1, List<Vector2f> shape2) {
        Vector2f d = new Vector2f(1, 0);

        List<Vector2f> simplex = new ArrayList<>();

        // Get the first Minkowski Difference point
        simplex.add(support(shape1, shape2, d));

        // Negate d for the next point
        d.negate();

        // Start looping
        while (true) {
            // Add a new point to the simplex because we haven't terminated yet
            simplex.add(support(shape1, shape2, d));

            // Make sure that the last point we added actually passed the origin
            if (simplex.get(simplex.size() - 1).dot(d) <= 0) {
                // If the point added last was not past the origin in the direction of d
                // then the Minkowski Sum cannot possibly contain the origin since
                // the last point added is on the edge of the Minkowski Difference
                return false;
            } else {
                // Otherwise we need to determine if the origin is in
                // the current simplex
                if (containsOrigin(simplex, d)) {
                    // If it does then we know there is a collision
                    return true;
                }
            }
        }
    }

    public static void main(String[] args) {
        List<Vector2f> shape1 = new ArrayList<>();
        List<Vector2f> shape2 = new ArrayList<>();

        shape1.add(new Vector2f(4, 5));
        shape1.add(new Vector2f(4, 11));
        shape1.add(new Vector2f(9, 9));

        shape2.add(new Vector2f(7, 3));
        shape2.add(new Vector2f(8, 12));

        System.out.println(polygonsIntersect(shape1, shape2));
    }

}
