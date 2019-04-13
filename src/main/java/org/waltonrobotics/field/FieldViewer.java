package org.waltonrobotics.field;

import org.waltonrobotics.geometry.ConvexHull;
import org.waltonrobotics.geometry.Vector2f;
import org.waltonrobotics.geometry.Vector2i;

import javax.swing.*;
import java.awt.*;
import java.util.ArrayList;

/**
 * Used for viewing obstacles in a field.
 */
public class FieldViewer extends JPanel {

    private int fieldWidthPixels;
    private int fieldHeightPixels;
    private FieldUnitConverter unitConverter;
    private ArrayList<ConvexHull> obstacles;
    private ArrayList<Polygon> polygons;

    /**
     * Creates a new field viewer panel.
     *
     * @param fieldWidthPixels:  Width of the field image in pixels
     * @param fieldHeightPixels: Height of the field image in pixels
     * @param unitConverter:     Unit converter for converting between pixel and real-world units
     * @param obstacles:         List of convex hull obstacles
     */
    public FieldViewer(int fieldWidthPixels, int fieldHeightPixels, FieldUnitConverter unitConverter, ArrayList<ConvexHull> obstacles) {
        this.fieldWidthPixels = fieldWidthPixels;
        this.fieldHeightPixels = fieldHeightPixels;
        this.unitConverter = unitConverter;
        this.obstacles = obstacles;
        this.polygons = new ArrayList<>();

        convertObstaclesToPolygons();
    }

    /**
     * Converts convex-hull obstacles with coordinates respective to origin in bottom-left corner to polygon objects with pixel coordinates.
     */
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

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);

        g.setColor(Color.BLUE);

        // Draw all polygons (obstacles)
        for (Polygon p : polygons) {
            g.fillPolygon(p);
        }
    }

    @Override
    public Dimension getPreferredSize() {
        return new Dimension(fieldWidthPixels, fieldHeightPixels);
    }

}
