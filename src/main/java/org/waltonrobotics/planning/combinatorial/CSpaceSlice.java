package org.waltonrobotics.planning.combinatorial;

import org.waltonrobotics.field.Field;
import org.waltonrobotics.geometry.ConvexHull;
import org.waltonrobotics.geometry.LineSegment;
import org.waltonrobotics.geometry.Vector2f;

import java.io.IOException;
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

        VisibilityGraph.calculateVisibilityGraph(vertices, edges, cSpaceObstacles);
    }

    public static void main(String[] args) {
        ConvexHull robot = new ConvexHull();

        robot.begin();
        robot.addPoint(new Vector2f(0.0, 0.0));
        robot.addPoint(new Vector2f(0.1, 0.0));
        robot.addPoint(new Vector2f(0.1, 0.1));
        robot.addPoint(new Vector2f(0.0, 0.1));
        robot.end();

        Field field = null;
        try {
            field = new Field("C:\\Users\\User\\Documents\\GitHub\\Path-Planning\\res\\fields\\DeepSpaceField.field");
        } catch (IOException | ClassNotFoundException e) {
            e.printStackTrace();
        }

        CSpaceSlice slice = new CSpaceSlice(robot, Math.toRadians(0.0), field.getObstacles());
    }

}
