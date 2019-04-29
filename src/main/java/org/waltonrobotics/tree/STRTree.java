package org.waltonrobotics.tree;

import org.waltonrobotics.geometry.Rectangle;

import java.util.Comparator;
import java.util.List;

public class STRTree {

    private int leafCapacity;
    private List<Rectangle> boundingBoxes;

    private static Comparator<Rectangle> xComparator = (r1, r2) -> r1.getCenterPoint().compareX(r2.getCenterPoint());
    private static Comparator<Rectangle> yComparator = (r1, r2) -> r1.getCenterPoint().compareY(r2.getCenterPoint());

    public STRTree(int leafCapacity, List<Rectangle> boundingBoxes) {
        this.leafCapacity = leafCapacity;
        this.boundingBoxes = boundingBoxes;
        calculateTree();
    }

    public int getLeafCapacity() {
        return leafCapacity;
    }

    public List<Rectangle> getBoundingBoxes() {
        return boundingBoxes;
    }

    private void calculateTree() {

    }

}
