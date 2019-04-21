package org.waltonrobotics.planning.combinatorial;

import org.waltonrobotics.field.Field;
import org.waltonrobotics.geometry.ConvexHull;
import org.waltonrobotics.geometry.Pose;
import org.waltonrobotics.geometry.Vector2f;
import org.waltonrobotics.planning.PathPlanner;

import javax.swing.*;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class CombinatorialPlanner implements PathPlanner {

    private ConfigurationSpace configurationSpace;
    private double rotationWeightTerm;

    private class Node {

        public Pose pose;
        public Node parent;
        public double fCost, gCost, hCost;

        public Node(Pose pose, Node parent, double gCost, double hCost) {
            this.pose = pose;
            this.parent = parent;
            this.gCost = gCost;
            this.hCost = hCost;
            this.fCost = this.gCost + this.hCost;
        }

    }

    private static Comparator<Node> nodeComparator = (n1, n2) -> {
        if (n2.fCost < n1.fCost) return 1;
        if (n2.fCost > n1.fCost) return -1;
        return 0;
    };

    private static class ClosestPoseComparator implements Comparator<Pose> {

        private Pose originalPose;

        public ClosestPoseComparator(Pose originalPose) {
            this.originalPose = originalPose;
        }

        @Override
        public int compare(Pose p1, Pose p2) {
            double p1DistanceToOriginalPose = p1.distance(originalPose);
            double p2DistanceToOriginalPose = p2.distance(originalPose);

            if (p1DistanceToOriginalPose < p2DistanceToOriginalPose) return -1;
            else if (p1DistanceToOriginalPose > p2DistanceToOriginalPose) return 1;

            return 0;
        }

    }

    public CombinatorialPlanner(ConvexHull robot, List<ConvexHull> obstacles, double angleResolutionDegrees, double rotationWeightTerm) {
        this.configurationSpace = new ConfigurationSpace(robot, obstacles, angleResolutionDegrees);
        this.rotationWeightTerm = rotationWeightTerm;
    }

    private double getPositiveCoterminalAngle(double angleDegrees) {
        return(angleDegrees %= 360) < 0 ? angleDegrees + 360 : angleDegrees;
    }

    private Pose getClosestPoseInSlice(VisibilityGraph slice, Pose pose) {
        double currentClosestDistance = Double.POSITIVE_INFINITY;
        Pose closestPose = null;

        for (Pose p : slice.getGraphAdjacencyList().keySet()) {
            double currentDistance = p.distance(pose);

            if (currentDistance < currentClosestDistance) {
                closestPose = p;
                currentClosestDistance = currentDistance;
            }
        }

        return closestPose;
    }

    @Override
    public List<Pose> findPath(Pose startingPose, Pose endingPose) {
        double startingAngle = getPositiveCoterminalAngle(startingPose.getDegrees());
        double endingAngle = getPositiveCoterminalAngle(endingPose.getDegrees());

        int startingSliceIndex = (int)Math.round(startingAngle / configurationSpace.getAngleResolutionDegrees());
        int endingSliceIndex = (int)Math.round(endingAngle / configurationSpace.getAngleResolutionDegrees());

        int lowestSliceIndex = 0;
        int highestSliceIndex = (int)(360.0 / configurationSpace.getAngleResolutionDegrees()) - 1;

        VisibilityGraph startingSlice = configurationSpace.getSlices().get(startingSliceIndex);
        VisibilityGraph endingSlice = configurationSpace.getSlices().get(endingSliceIndex);

        Pose closestStartingPose = getClosestPoseInSlice(startingSlice, startingPose);
        Pose closestEndingPose = getClosestPoseInSlice(endingSlice, endingPose);

        List<Node> openList = new ArrayList<>();
        List<Node> closedList = new ArrayList<>();

        Node current = new Node(closestStartingPose, null, 0, heuristic(startingPose, endingPose));
        openList.add(current);

        while (!openList.isEmpty()) {
            openList.sort(nodeComparator);

            current = openList.get(0);

            if (current.pose.equals(closestEndingPose)) {
                List<Pose> path = new ArrayList<>();

                path.add(endingPose);

                while (current.parent != null) {
                    path.add(current.pose);
                    current = current.parent;
                }

                path.add(startingPose);

                Collections.reverse(path);

                return path;
            }

            openList.remove(current);
            closedList.add(current);

            int currentSliceIndex = (int)Math.round(current.pose.getDegrees() / configurationSpace.getAngleResolutionDegrees());
            int belowSliceIndex = (currentSliceIndex - 1) < lowestSliceIndex ? highestSliceIndex : (currentSliceIndex - 1);
            int aboveSliceIndex = (currentSliceIndex + 1) > highestSliceIndex ? lowestSliceIndex : (currentSliceIndex + 1);

            VisibilityGraph currentSlice = configurationSpace.getSlices().get(currentSliceIndex);
            VisibilityGraph belowSlice = configurationSpace.getSlices().get(belowSliceIndex);
            VisibilityGraph aboveSlice = configurationSpace.getSlices().get(aboveSliceIndex);

            for (Pose p : currentSlice.getGraphAdjacencyList().get(current.pose)) {
                exploreNode(openList, closedList, current, p, endingPose);
            }

            exploreNode(openList, closedList, current, getClosestPoseInSlice(belowSlice, current.pose), endingPose);
            exploreNode(openList, closedList, current, getClosestPoseInSlice(aboveSlice, current.pose), endingPose);
        }

        return null;
    }

    private void exploreNode(List<Node> openList, List<Node> closedList, Node current, Pose next, Pose endingPose) {
        double gCost = current.gCost + heuristic(current.pose, next);
        double hCost = heuristic(next, endingPose);
        Node node = new Node(next, current, gCost, hCost);
        if (poseInList(closedList, next) && gCost >= current.gCost) return;
        if (!poseInList(closedList, next) || gCost < current.gCost) openList.add(node);
    }

    private boolean poseInList(List<Node> list, Pose pose) {
        for (Node n : list) {
            if (n.pose.equals(pose)) return true;
        }
        return false;
    }

    private double heuristic(Pose a, Pose b) {
        double aAngle = getPositiveCoterminalAngle(a.getDegrees());
        double bAngle = getPositiveCoterminalAngle(b.getDegrees());

        return Math.sqrt(Math.pow(a.getX() - b.getX(), 2) +
                Math.pow(a.getY() - b.getY(), 2) +
                rotationWeightTerm * Math.pow(aAngle - bAngle, 2));
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

        CombinatorialPlanner planner = new CombinatorialPlanner(robot, field.getObstacles(), 45, 1);

        List<Pose> path = planner.findPath(new Pose(7.614804292929293, 6.618474759274993, 90), new Pose(15, 3, 180));

        System.out.println(path);
        JFrame fieldGUIFrame = new JFrame();

        fieldGUIFrame.setResizable(false);
        fieldGUIFrame.setTitle("Generated Field");
        fieldGUIFrame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);

        //fieldGUIFrame.add(new FieldViewer(field.getFieldWidthPixels(), field.getFieldHeightPixels(), field.getUnitConverter(), field.getObstacles()));
        fieldGUIFrame.add(new PathViewer(field.getFieldWidthPixels(), field.getFieldHeightPixels(), field.getUnitConverter(), field.getObstacles(), path));

        fieldGUIFrame.pack();

        // Display the field
        fieldGUIFrame.setVisible(true);
    }

}
