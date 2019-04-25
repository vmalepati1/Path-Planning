package org.waltonrobotics.planning.combinatorial;

import org.waltonrobotics.field.Field;
import org.waltonrobotics.geometry.ConvexHull;
import org.waltonrobotics.geometry.LineSegment;
import org.waltonrobotics.geometry.Vector2f;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class ConfigurationSpace {

    private ConvexHull robot;
    private List<ConvexHull> obstacles;
    private double angleResolutionDegrees;

    private List<VisibilityGraph> slices;

    public ConfigurationSpace(ConvexHull robot, List<ConvexHull> obstacles, double angleResolutionDegrees) {
        this.robot = robot;
        this.obstacles = obstacles;
        this.angleResolutionDegrees = angleResolutionDegrees;

        calculateSlices();
    }

    private void calculateSlices() {
        if (angleResolutionDegrees < 0 || angleResolutionDegrees > 360) {
            throw new IllegalArgumentException("The angle resolution measured in degrees must be greater than 0 and less than 360!");
        }

        slices = new ArrayList<>();

        for (double angle = 0; angle <= 360; angle += angleResolutionDegrees) {
            if (angle == 360) break;

            double angleRadians = Math.toRadians(angle);

            List<ConvexHull> cSpaceObstacles = new ArrayList<>();

            ConvexHull transformedRobot = new ConvexHull();

            transformedRobot.begin();

            for (Vector2f v : robot.getConvexPoints()) {
                double xPrime = (v.getX() * Math.cos(angleRadians)) - (v.getY() * Math.sin(angleRadians));
                double yPrime = (v.getX() * Math.sin(angleRadians)) + (v.getY() * Math.cos(angleRadians));

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

            slices.add(new VisibilityGraph(vertices, edges, cSpaceObstacles, angle));
        }
    }

    public ConvexHull getRobot() {
        return robot;
    }

    public List<ConvexHull> getObstacles() {
        return obstacles;
    }

    public double getAngleResolutionDegrees() {
        return angleResolutionDegrees;
    }

    public List<VisibilityGraph> getSlices() {
        return slices;
    }

    public static void main(String[] args) {
        Field field = null;

        try {
            field = new Field("C:\\Users\\User\\Documents\\GitHub\\Path-Planning\\res\\fields\\DeepSpaceField.field");
        } catch (IOException | ClassNotFoundException e) {
            e.printStackTrace();
        }

        ConvexHull robot = new ConvexHull();

        robot.begin();
        robot.addPoint(new Vector2f(0.0, 0.0));
        robot.addPoint(new Vector2f(1.0, 0.0));
        robot.addPoint(new Vector2f(1.0, 1.0));
        robot.addPoint(new Vector2f(0.0, 1.0));
        robot.end();

        ConfigurationSpace cSpace = new ConfigurationSpace(robot, field.getObstacles(),5.0);
    }

}
