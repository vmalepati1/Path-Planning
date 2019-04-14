package org.waltonrobotics.planning.combinatorial;

import org.waltonrobotics.geometry.ConvexHull;
import org.waltonrobotics.geometry.LineSegment;
import org.waltonrobotics.geometry.Vector2f;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class CSpaceSlice {

    private ConvexHull robot;
    private double robotOrientationRadians;
    private List<ConvexHull> obstacles;

    public CSpaceSlice(ConvexHull robot, double robotOrientationRadians, List<ConvexHull> obstacles) {
        this.robot = robot;
        this.robotOrientationRadians = robotOrientationRadians;
        this.obstacles = obstacles;

        calculateNodeGraph();
    }

    private void calculateNodeGraph() {
        List<ConvexHull> cSpaceObstacles = new ArrayList<>();

        ConvexHull transformedRobot = new ConvexHull();

        transformedRobot.begin();

        for (Vector2f v : robot.getConvexPoints()) {
            double xPrime = (v.getX() * Math.cos(robotOrientationRadians)) - (v.getY() * Math.sin(robotOrientationRadians));
            double yPrime = (v.getX() * Math.sin(robotOrientationRadians)) + (v.getY() * Math.cos(robotOrientationRadians));

            transformedRobot.addPoint(new Vector2f(xPrime, yPrime));
        }

        transformedRobot.end();

        int polygonID = 0;

        for (ConvexHull cvh : obstacles) {
            ConvexHull cSpaceObstacle = new ConvexHull();

            cSpaceObstacle.begin();

            for (Vector2f obstacleV : cvh.getConvexPoints()) {
                for (Vector2f robotV : transformedRobot.getConvexPoints()) {
                    cSpaceObstacle.addPoint(new Vector2f(obstacleV.getX() - robotV.getX(), obstacleV.getY() - robotV.getY(), polygonID));
                }
            }

            cSpaceObstacle.end();

            polygonID++;

            cSpaceObstacles.add(cSpaceObstacle);
        }

        // Get all the obstacle vertices into one list
        List<Vector2f> vertices = cSpaceObstacles.stream().flatMap(x -> x.getConvexPoints().stream()).collect(Collectors.toList());
        List<LineSegment> edges = cSpaceObstacles.stream().flatMap(x -> x.getEdges().stream()).collect(Collectors.toList());

        List<Vector2f> nodes = VisibilityGraph.calculateVisibilityGraph(vertices, edges, cSpaceObstacles);
    }

    public static void main(String[] args) {
        ConvexHull robot = new ConvexHull();

        robot.begin();
        robot.addPoint(new Vector2f(0.0, 0.0));
        robot.addPoint(new Vector2f(1.0, 0.0));
        robot.addPoint(new Vector2f(1.0, 1.0));
        robot.addPoint(new Vector2f(0.0, 1.0));
        robot.end();

        List<ConvexHull> obstacles = new ArrayList<>();

        ConvexHull obstacle = new ConvexHull();

        obstacle.begin();
        obstacle.addPoint(new Vector2f(3.0, 3.0));
        obstacle.addPoint(new Vector2f(4.0, 3.0));
        obstacle.addPoint(new Vector2f(4.0, 4.0));
        obstacle.addPoint(new Vector2f(3.0, 4.0));
        obstacle.end();

        obstacles.add(obstacle);

        ConvexHull obstacle2 = new ConvexHull();
        obstacle2.begin();
        obstacle2.addPoint(new Vector2f(8.0, 8.0));
        obstacle2.addPoint(new Vector2f(9.0, 8.0));
        obstacle2.addPoint(new Vector2f(9.0, 9.0));
        obstacle2.addPoint(new Vector2f(8.0, 9.0));
        obstacle2.end();

        obstacles.add(obstacle2);

        CSpaceSlice slice = new CSpaceSlice(robot, Math.toRadians(0.0), obstacles);
    }

}
