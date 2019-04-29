package org.waltonrobotics.planning.combinatorial;

import org.waltonrobotics.field.FieldUnitConverter;
import org.waltonrobotics.geometry.ConvexHull;
import org.waltonrobotics.geometry.Pose;
import org.waltonrobotics.geometry.Vector2f;
import org.waltonrobotics.geometry.Vector2i;

import javax.swing.*;
import java.awt.*;
import java.util.ArrayList;
import java.util.List;

public class PathViewer extends JPanel {

    private int fieldWidthPixels;
    private int fieldHeightPixels;
    private FieldUnitConverter unitConverter;
    private List<ConvexHull> obstacles;
    private ArrayList<Polygon> polygons;
    private List<Pose> path;

    public PathViewer(int fieldWidthPixels, int fieldHeightPixels, FieldUnitConverter unitConverter, List<ConvexHull> obstacles, List<Pose> path) {
        this.fieldWidthPixels = fieldWidthPixels;
        this.fieldHeightPixels = fieldHeightPixels;
        this.unitConverter = unitConverter;
        this.obstacles = obstacles;
        this.polygons = new ArrayList<>();
        this.path = path;

        convertObstaclesToPolygons();
    }

    private void convertObstaclesToPolygons() {
        for (ConvexHull cvh : obstacles) {
            Polygon p = new Polygon();

            for (Vector2f v : cvh.getConvexPoints()) {
                Vector2i pixelPoint = unitConverter.convertActualPointToPixelPoint(v, fieldHeightPixels);

                p.addPoint(pixelPoint.getX(), pixelPoint.getY());
            }

            polygons.add(p);
        }
    }

    protected void paintComponent(Graphics g) {
        super.paintComponent(g);

        g.setColor(Color.BLACK);

        // Draw all polygons (obstacles)
        for (Polygon p : polygons) {
            g.fillPolygon(p);
        }

        for (int i = 1; i < path.size(); i++) {
            Vector2i pixelPoint = unitConverter.convertActualPointToPixelPoint(new Vector2f(path.get(i).getX(), path.get(i).getY()), fieldHeightPixels);
            Vector2i prevPixelPoint = unitConverter.convertActualPointToPixelPoint(new Vector2f(path.get(i - 1).getX(), path.get(i - 1).getY()), fieldHeightPixels);

            g.setColor(Color.RED);
            //g.fillOval(prevPixelPoint.getX() - 13, prevPixelPoint.getY() - 13, 26, 26);

            g.setColor(Color.BLUE);
            g.drawLine(pixelPoint.getX(), pixelPoint.getY(), prevPixelPoint.getX(), prevPixelPoint.getY());
        }

        for (Pose p : path) {
            List<Vector2f> robotPoints = new ArrayList<>();

            robotPoints.add(new Vector2f(p.getX() - 0.15, p.getY() - 0.15));
            robotPoints.add(new Vector2f(p.getX() + 0.15, p.getY() - 0.15));
            robotPoints.add(new Vector2f(p.getX() + 0.15, p.getY() + 0.15));
            robotPoints.add(new Vector2f(p.getX() - 0.15, p.getY() + 0.15));

            List<Vector2i> pixelPoints = new ArrayList<>();

            for (Vector2f v : robotPoints) {
                double angleRadians = Math.toRadians(p.getDegrees());
                double xPrime = Math.cos(angleRadians) * (v.getX() - p.getX()) - Math.sin(angleRadians) * (v.getY() - p.getY()) + p.getX();
                double yPrime = Math.sin(angleRadians) * (v.getX() - p.getX()) + Math.cos(angleRadians) * (v.getY() - p.getY()) + p.getY();

                System.out.println(xPrime + ", " + yPrime);
                Vector2i pixelPoint = unitConverter.convertActualPointToPixelPoint(new Vector2f(xPrime, yPrime), fieldHeightPixels);
                pixelPoints.add(pixelPoint);
            }

            g.setColor(Color.ORANGE);
            g.drawLine(pixelPoints.get(0).getX(), pixelPoints.get(0).getY(), pixelPoints.get(pixelPoints.size() - 1).getX(), pixelPoints.get(pixelPoints.size() - 1).getY());

            for (int i = 1; i < pixelPoints.size(); i++) {
                Vector2i prevPoint = pixelPoints.get(i - 1);
                Vector2i point = pixelPoints.get(i);

                //System.out.println(point);

                g.setColor(Color.ORANGE);
                g.drawLine(prevPoint.getX(), prevPoint.getY(), point.getX(), point.getY());
            }
        }
    }

    @Override
    public Dimension getPreferredSize() {
        return new Dimension(fieldWidthPixels, fieldHeightPixels);
    }


}
