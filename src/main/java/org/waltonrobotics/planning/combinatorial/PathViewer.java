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
import java.util.Random;

public class PathViewer extends JPanel {

    private int fieldWidthPixels;
    private int fieldHeightPixels;
    private FieldUnitConverter unitConverter;
    private ArrayList<ConvexHull> obstacles;
    private ArrayList<Polygon> polygons;
    private List<Pose> path;

    public PathViewer(int fieldWidthPixels, int fieldHeightPixels, FieldUnitConverter unitConverter, ArrayList<ConvexHull> obstacles, List<Pose> path) {
        this.fieldWidthPixels = fieldWidthPixels;
        this.fieldHeightPixels = fieldHeightPixels;
        this.unitConverter = unitConverter;
        this.obstacles = obstacles;
        this.polygons = new ArrayList<>();
        this.path = path;

        System.out.println(path.size());

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

        g.setColor(Color.RED);

        for (Pose p : path) {
            Vector2i pixelPoint = unitConverter.convertActualPointToPixelPoint(new Vector2f(p.getX(), p.getY()), fieldHeightPixels);
            g.fillRect(pixelPoint.getX(), pixelPoint.getY(), 20, 20);
        }

        for (int i = 1; i < path.size(); i++) {
            Random randomGenerator = new Random();
            int red = randomGenerator.nextInt(256);
            int green = randomGenerator.nextInt(256);
            int blue = randomGenerator.nextInt(256);

            Color randomColour = new Color(red,green,blue);

            g.setColor(randomColour);

            Vector2i pixelPoint = unitConverter.convertActualPointToPixelPoint(new Vector2f(path.get(i).getX(), path.get(i).getY()), fieldHeightPixels);
            Vector2i prevPixelPoint = unitConverter.convertActualPointToPixelPoint(new Vector2f(path.get(i - 1).getX(), path.get(i - 1).getY()), fieldHeightPixels);

            g.drawLine(pixelPoint.getX(), pixelPoint.getY(), prevPixelPoint.getX(), prevPixelPoint.getY());
        }
    }

    @Override
    public Dimension getPreferredSize() {
        return new Dimension(fieldWidthPixels, fieldHeightPixels);
    }


}
