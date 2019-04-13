package org.waltonrobotics.field;

import org.waltonrobotics.geometry.Vector2f;
import org.waltonrobotics.geometry.Vector2i;

import java.io.Serializable;

/**
 * Used to convert between pixel field units and real-world units (feet, meters, etc.).
 */
public class FieldUnitConverter implements Serializable {

    // Pixels to actual real-world units conversion factor for field image
    private double pixelsToActualUnits;
    // Real-world units to pixels conversion factor for field image
    private double actualUnitsToPixels;

    /**
     * @param pixels:      Arbitrary amount of pixels in image
     * @param actualUnits: How many actual real-world units are in the amount of pixels specified
     */
    public FieldUnitConverter(int pixels, double actualUnits) {
        pixelsToActualUnits = actualUnits / pixels;
        actualUnitsToPixels = pixels / actualUnits;
    }

    /**
     * @param pixelsToActualUnits: Pixels to real-world units conversion factor
     */
    public FieldUnitConverter(double pixelsToActualUnits) {
        this.pixelsToActualUnits = pixelsToActualUnits;
        this.actualUnitsToPixels = 1 / pixelsToActualUnits;
    }

    /**
     * @return Pixels to actual real-world units conversion factor for field image
     */
    public double getPixelsToActualUnits() {
        return pixelsToActualUnits;
    }

    /**
     * @return Real-world units to pixels conversion factor for field image
     */
    public double getActualUnitsToPixels() {
        return actualUnitsToPixels;
    }

    /**
     * @param pixels: Amount of pixels in image
     * @return Real-world unit equivalent
     */
    public double convertPixelsToActualUnits(int pixels) {
        return pixels * pixelsToActualUnits;
    }

    /**
     * @param actualUnits: Real-world units
     * @return Amount of pixels in image equivalent
     */
    public int convertActualUnitsToPixels(double actualUnits) {
        return (int) Math.round(actualUnits * actualUnitsToPixels);
    }

    /**
     * @param pixelPoint:        A pixel coordinate on the image
     * @param translatedOrigin:  Translated origin of where the field's top left corner is in pixel coordinates
     * @param fieldHeightPixels: Height of field in pixels from the translated origin
     * @return Real-world unit vector in respect to origin in the bottom-left corner
     */
    public Vector2f convertPixelPointToActualPoint(Vector2i pixelPoint, Vector2i translatedOrigin, int fieldHeightPixels) {
        return new Vector2f(convertPixelsToActualUnits(pixelPoint.getX() - translatedOrigin.getX()),
                convertPixelsToActualUnits(fieldHeightPixels - (pixelPoint.getY() - translatedOrigin.getY())));
    }

    /**
     * @param actualPoint:       A coordinate in real-world units in respect to origin in the bottom-left corner
     * @param fieldHeightPixels: Height of field in pixels
     * @return Pixel coordinate equivalent
     */
    public Vector2i convertActualPointToPixelPoint(Vector2f actualPoint, int fieldHeightPixels) {
        return new Vector2i(convertActualUnitsToPixels(actualPoint.getX()), fieldHeightPixels - convertActualUnitsToPixels(actualPoint.getY()));
    }

}