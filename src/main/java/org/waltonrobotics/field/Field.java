package org.waltonrobotics.field;

import org.waltonrobotics.geometry.ConvexHull;

import java.io.FileInputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.util.ArrayList;
import java.util.List;

public class Field {

    private int fieldWidthPixels;
    private int fieldHeightPixels;
    private double fieldWidthActualUnits;
    private double fieldHeightActualUnits;
    private ArrayList<ConvexHull> obstacles;
    private FieldUnitConverter unitConverter;

    public Field(String path) throws IOException, ClassNotFoundException {
        this.obstacles = new ArrayList<>();

        loadFromFile(path);
    }

    public int getFieldWidthPixels() {
        return fieldWidthPixels;
    }

    public int getFieldHeightPixels() {
        return fieldHeightPixels;
    }

    public double getFieldWidthActualUnits() {
        return fieldWidthActualUnits;
    }

    public double getFieldHeightActualUnits() {
        return fieldHeightActualUnits;
    }

    public ArrayList<ConvexHull> getObstacles() {
        return obstacles;
    }

    public FieldUnitConverter getUnitConverter() {
        return unitConverter;
    }

    public void loadFromFile(String path) throws IOException, ClassNotFoundException {
        FileInputStream fis = new FileInputStream(path);
        ObjectInputStream ois = new ObjectInputStream(fis);

        fieldWidthPixels = ois.readInt();
        fieldHeightPixels = ois.readInt();
        fieldWidthActualUnits = ois.readDouble();
        fieldHeightActualUnits = ois.readDouble();

        obstacles.clear();

        List<?> supposedObstacleList = (List<?>) ois.readObject();
        for (Object object : supposedObstacleList) {
            if (object instanceof ConvexHull) {
                obstacles.add((ConvexHull) object);
            }
        }

        unitConverter = (FieldUnitConverter) ois.readObject();

        ois.close();
    }

}
