package org.waltonrobotics.field;

import org.waltonrobotics.geometry.ConvexHull;
import org.waltonrobotics.geometry.Vector2f;
import org.waltonrobotics.geometry.Vector2i;

import javax.imageio.ImageIO;
import javax.swing.*;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectOutputStream;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Queue;

/**
 * Used to detect obstacles within a field image using blob detection and write them to a field file or display them.
 */
public class FieldGenerator {

    private String fieldDrawingPath;

    private Color freeSpaceColor;

    private int freeSpaceChannelTolerance;
    private int blobColorChannelTolerance;
    private double borderDetectionFactor;

    private Vector2i translatedOrigin;

    private int fieldWidthPixels;
    private int fieldHeightPixels;

    private double fieldWidthActualUnits;
    private double fieldHeightActualUnits;

    private BufferedImage fieldDrawing;

    private ArrayList<ConvexHull> obstacles;
    private FieldUnitConverter unitConverter;

    /**
     * Creates a new field generator. The generated field can be displayed or saved to a field file.
     *
     * @param path:                      File path to a field image
     * @param freeSpaceColor:            The color within the field image where there are no obstacles (ground color)
     * @param freeSpaceChannelTolerance: How much deviation is allowed from free space color for non obstacles
     * @param blobColorChannelTolerance: How much deviation is allowed for obstacle blob detection
     * @param borderDetectionFactor:     How many times more an obstacle's area compared to its border
     *                                   area needs to be for it to be considered a border
     * @param fieldWidthActualUnits:     The width of the actual field in real-world units (ex. meters)
     * @param fieldHeightActualUnits:    The height of the actual field in real-world units (ex. meters)
     * @throws IllegalArgumentException: User-provided arguments are invalid
     * @throws IOException:              Unable to open image file from given path
     */
    public FieldGenerator(String path,
                          Color freeSpaceColor,
                          int freeSpaceChannelTolerance, int blobColorChannelTolerance,
                          double borderDetectionFactor,
                          double fieldWidthActualUnits, double fieldHeightActualUnits) throws IllegalArgumentException, IOException {
        this.fieldDrawingPath = path;
        this.freeSpaceColor = freeSpaceColor;
        this.freeSpaceChannelTolerance = freeSpaceChannelTolerance;
        this.blobColorChannelTolerance = blobColorChannelTolerance;
        this.borderDetectionFactor = borderDetectionFactor;
        this.translatedOrigin = new Vector2i();
        this.fieldWidthPixels = -1;
        this.fieldHeightPixels = -1;
        this.fieldWidthActualUnits = fieldWidthActualUnits;
        this.fieldHeightActualUnits = fieldHeightActualUnits;
        this.obstacles = new ArrayList<>();

        loadFieldFromFile();
    }

    /**
     * Creates a new field generator. The generated field can be displayed or saved to a field file.
     *
     * @param path:                      File path to a field image
     * @param freeSpaceColor:            The color within the field image where there are no obstacles (ground color)
     * @param freeSpaceChannelTolerance: How much deviation is allowed from free space color for non obstacles
     * @param blobColorChannelTolerance: How much deviation is allowed for obstacle blob detection
     * @param borderDetectionFactor      How many times more an obstacle's area compared to its border
     *                                   area needs to be for it to be considered a border
     * @param translatedOrigin:          The pixel coordinate of where the field's top-left corner is (in respect to the top-left of the field image)
     * @param fieldWidthPixels:          The width of the field in pixels from the translated origin
     * @param fieldHeightPixels          The height of the field in pixels from the translated origin
     * @param fieldWidthActualUnits:     The width of the actual field in real-world units (ex. meters)
     * @param fieldHeightActualUnits:    The height of the actual field in real-world units (ex. meters)
     * @throws IllegalArgumentException: User-provided arguments are invalid
     * @throws IOException:              Unable to open image file from given path
     */
    public FieldGenerator(String path,
                          Color freeSpaceColor,
                          int freeSpaceChannelTolerance, int blobColorChannelTolerance,
                          double borderDetectionFactor,
                          Vector2i translatedOrigin,
                          int fieldWidthPixels, int fieldHeightPixels,
                          double fieldWidthActualUnits, double fieldHeightActualUnits) throws IllegalArgumentException, IOException {
        this.fieldDrawingPath = path;
        this.freeSpaceColor = freeSpaceColor;
        this.freeSpaceChannelTolerance = freeSpaceChannelTolerance;
        this.blobColorChannelTolerance = blobColorChannelTolerance;
        this.borderDetectionFactor = borderDetectionFactor;
        this.translatedOrigin = translatedOrigin;
        this.fieldWidthPixels = fieldWidthPixels;
        this.fieldHeightPixels = fieldHeightPixels;
        this.fieldWidthActualUnits = fieldWidthActualUnits;
        this.fieldHeightActualUnits = fieldHeightActualUnits;
        this.obstacles = new ArrayList<>();

        loadFieldFromFile();
    }

    public static void main(String[] args) {
        FieldGenerator testFieldGenerator = null;
        try {
            testFieldGenerator = new FieldGenerator("C:\\Users\\Vikas Malepati\\Documents\\GitHub\\Path-Planning\\res\\fields\\DeepSpaceFieldDrawing.png",
                    new Color(120, 120, 120),
                    20, 40,
                    0.1,
                    new Vector2i(131, 143),
                    1584, 642,
                    22.54, 9.14);
        } catch (IllegalArgumentException | IOException e) {
            e.printStackTrace();
        }

        testFieldGenerator.displayField();

        try {
            testFieldGenerator.saveFieldToFile("C:\\Users\\Vikas Malepati\\Documents\\GitHub\\Path-Planning\\res\\fields\\DeepSpaceField.field");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Displays the generated field in a new window.
     */
    public void displayField() {
        JFrame fieldGUIFrame = new JFrame();

        fieldGUIFrame.setResizable(false);
        fieldGUIFrame.setTitle("Generated Field");
        fieldGUIFrame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);

        fieldGUIFrame.add(new FieldViewer(fieldWidthPixels, fieldHeightPixels, unitConverter, obstacles));

        fieldGUIFrame.pack();

        // Display the field
        fieldGUIFrame.setVisible(true);
    }

    /**
     * Saves the generated field to a file to be used for a Field object.
     *
     * @param path: Path to non-existing or already-existing field file
     * @throws IOException: Unable to open file for writing
     */
    public void saveFieldToFile(String path) throws IOException {
        FileOutputStream fos = new FileOutputStream(path);
        ObjectOutputStream oos = new ObjectOutputStream(fos);
        oos.writeInt(fieldWidthPixels);
        oos.writeInt(fieldHeightPixels);
        oos.writeDouble(fieldWidthActualUnits);
        oos.writeDouble(fieldHeightActualUnits);
        oos.writeObject(obstacles);
        oos.writeObject(unitConverter);
        oos.close();
    }

    /**
     * Loads the field from an image file and determine obstacles through blob detection.
     *
     * @throws IllegalArgumentException: User-provided arguments are invalid
     * @throws IOException:              Unable to open image file from given path
     */
    private void loadFieldFromFile() throws IllegalArgumentException, IOException {
        // Preliminary variable checking
        if (freeSpaceChannelTolerance < 0 || freeSpaceChannelTolerance > 255
                || blobColorChannelTolerance < 0 || blobColorChannelTolerance > 255) {
            throw new IllegalArgumentException("Tolerances must be in between 0 and 255!");
        }

        if (borderDetectionFactor < 0) {
            throw new IllegalArgumentException("Border detection factor must be greater than 0!");
        }

        // Read in the image file
        fieldDrawing = ImageIO.read(new File(fieldDrawingPath));

        if (fieldWidthPixels + translatedOrigin.getX() > fieldDrawing.getWidth()) {
            throw new IllegalArgumentException("Specified field width in pixels is larger than the width of the field image in pixels!");
        }

        if (fieldHeightPixels + translatedOrigin.getY() > fieldDrawing.getHeight()) {
            throw new IllegalArgumentException("Specified field height in pixels is larger than the height of the field image in pixels!");
        }

        if (fieldWidthPixels < 0) {
            fieldWidthPixels = fieldDrawing.getWidth();
        }

        if (fieldHeightPixels < 0) {
            fieldHeightPixels = fieldDrawing.getHeight();
        }

        // Determine our conversion factors for pixels to real-world units for the x and y axes
        double conversionFactorWidth = fieldWidthActualUnits / fieldWidthPixels;
        double conversionFactorHeight = fieldHeightActualUnits / fieldHeightPixels;

        // The conversion factor from pixels to real-world units is the average between conversion factors for x and y axes
        unitConverter = new FieldUnitConverter((conversionFactorWidth + conversionFactorHeight) / 2);

        // Run blob detection and fill our convex hull obstacle list
        generateObstaclesFromImage();
    }

    /**
     * Checks if two colors are similar using a given channel tolerance that is used on the red, green, and blue channels.
     *
     * @param a:                First color
     * @param b:                Second color
     * @param channelTolerance: Maximum deviance allowed for red, green, and blue channels for two colors to be determined as similar
     * @return Whether two colors are similar
     */
    private boolean areColorsClose(Color a, Color b, int channelTolerance) {
        // Find our deviations on each color channel
        int redDeviation = Math.abs(b.getRed() - a.getRed());
        int greenDeviation = Math.abs(b.getGreen() - a.getGreen());
        int blueDeviation = Math.abs(b.getBlue() - a.getBlue());

        // Return true if colors are similar, false otherwise
        return (redDeviation <= channelTolerance
                && greenDeviation <= channelTolerance
                && blueDeviation <= channelTolerance);
    }

    /**
     * Uses blob detection to find obstacles within the field image and populates convex hull array with the obstacles.
     */
    private void generateObstaclesFromImage() {
        // Pixels we have visited
        boolean[][] painted = new boolean[fieldDrawing.getHeight()][fieldDrawing.getWidth()];

        for (int i = 0; i < fieldDrawing.getHeight(); i++) {
            for (int j = 0; j < fieldDrawing.getWidth(); j++) {

                // Our current pixel's color
                Color originalBlobColor = new Color(fieldDrawing.getRGB(j, i));

                // If we haven't visited this pixel yet
                if (!painted[i][j]) {

                    // Create a new convex hull for a possible obstacle
                    ConvexHull newObstacleArea = new ConvexHull();

                    // Holds pixel points of the blob
                    Queue<Point> queue = new LinkedList<>();
                    queue.add(new Point(j, i));

                    // Begin constructing our convex hull
                    newObstacleArea.begin();

                    // How many pixels are in the blob
                    int pixelCount = 0;

                    while (!queue.isEmpty()) {
                        Point p = queue.remove();

                        // If we are within the field range
                        if ((p.x >= translatedOrigin.getX())
                                && (p.x <= translatedOrigin.getX() + fieldWidthPixels)
                                && (p.y >= translatedOrigin.getY())
                                && (p.y <= translatedOrigin.getY() + fieldHeightPixels)) {

                            // We are performing a paint fill
                            Color searchColor = new Color(fieldDrawing.getRGB(p.x, p.y));

                            // If the current color is not part of the free space and is close to the blob color
                            if (!painted[p.y][p.x]
                                    && areColorsClose(searchColor, originalBlobColor, blobColorChannelTolerance)
                                    && !areColorsClose(searchColor, freeSpaceColor, freeSpaceChannelTolerance)) {
                                painted[p.y][p.x] = true;

                                // Convert the pixel coordinate to a normal cartesian plane coordinate in real-world units in which (0, 0) is in the bottom-left corner
                                Vector2f actualPoint = unitConverter.convertPixelPointToActualPoint(new Vector2i(p.x, p.y), translatedOrigin, fieldHeightPixels);

                                // Add the real-world coordinate to our convex hull
                                newObstacleArea.addPoint(actualPoint);

                                // Increment pixel count
                                pixelCount++;

                                queue.add(new Point(p.x + 1, p.y));
                                queue.add(new Point(p.x - 1, p.y));
                                queue.add(new Point(p.x, p.y + 1));
                                queue.add(new Point(p.x, p.y - 1));
                            }
                        }
                    }

                    // If we have more than three points in our blob, start calculation of convex hull
                    if (pixelCount >= 3) {
                        // Calculate convex hull
                        newObstacleArea.end();

                        // Border check: If the inner area of the obstacle is much greater than the pixels in the blob (if the obstacle is a border),
                        // ignore it.
                        if (unitConverter.convertActualUnitsToPixels(newObstacleArea.getArea()) < (pixelCount * borderDetectionFactor)) {
                            obstacles.add(newObstacleArea);
                        }
                    }
                }
            }
        }
    }

}