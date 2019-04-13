package org.waltonrobotics.planning;

import org.waltonrobotics.geometry.ConvexHull;
import org.waltonrobotics.geometry.Vector2f;

import java.util.ArrayList;
import java.util.List;

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

        for (ConvexHull cvh : obstacles) {
            ConvexHull cSpaceObstacle = new ConvexHull();

            cSpaceObstacle.begin();

            for (Vector2f obstacleV : cvh.getConvexPoints()) {
                for (Vector2f robotV : transformedRobot.getConvexPoints()) {
                    cSpaceObstacle.addPoint(new Vector2f(obstacleV.getX() - robotV.getX(), obstacleV.getY() - robotV.getY()));
                }
            }

            cSpaceObstacle.end();

            cSpaceObstacles.add(cSpaceObstacle);
        }


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
        obstacle.addPoint(new Vector2f(2.0, 2.0));
        obstacle.addPoint(new Vector2f(3.0, 2.0));
        obstacle.addPoint(new Vector2f(3.0, 3.0));
        obstacle.addPoint(new Vector2f(2.0, 3.0));
        obstacle.end();

        obstacles.add(obstacle);

        CSpaceSlice slice = new CSpaceSlice(robot, Math.toRadians(0.0), obstacles);
    }

}
