package org.waltonrobotics.tree;

import org.waltonrobotics.geometry.ConvexHull;
import org.waltonrobotics.geometry.Rectangle;
import org.waltonrobotics.geometry.Vector2f;

import java.util.*;

public class STRTree {

    private static Comparator<Rectangle> xComparator = (r1, r2) -> r1.getCenterPoint().compareX(r2.getCenterPoint());
    private static Comparator<Rectangle> yComparator = (r1, r2) -> r1.getCenterPoint().compareY(r2.getCenterPoint());
    private int leafCapacity;
    private List<Rectangle> boundingBoxes;
    private STRTreeNode root;

    public STRTree(int leafCapacity, List<Rectangle> boundingBoxes) {
        this.leafCapacity = leafCapacity;
        this.boundingBoxes = boundingBoxes;
        calculateTree();
    }

    public static void main(String[] args) {
        ConvexHull a = new ConvexHull();
        a.begin();
        a.addPoint(new Vector2f(0, 0));
        a.addPoint(new Vector2f(5, 0));
        a.addPoint(new Vector2f(5, 1));
        a.addPoint(new Vector2f(2, 1));
        a.addPoint(new Vector2f(3, 2));
        a.addPoint(new Vector2f(0, 2));
        a.addPoint(new Vector2f(2, 0));
        a.addPoint(new Vector2f(3, 0));
        a.end();

        System.out.println(a.getConvexPoints());

        List<Rectangle> boundingBoxes = new ArrayList<>();

        boundingBoxes.add(new Rectangle(0, 1, 0, 1));
        boundingBoxes.add(new Rectangle(-2, 2, -2, 2));
        boundingBoxes.add(new Rectangle(4, 5, 4, 5));
        boundingBoxes.add(new Rectangle(6, 7, 6, 7));
        boundingBoxes.add(new Rectangle(8, 9, 8, 9));
        boundingBoxes.add(new Rectangle(10, 11, 10, 11));
        boundingBoxes.add(new Rectangle(12, 13, 12, 13));

        STRTree strTree = new STRTree(2, boundingBoxes);

        System.out.println(strTree.getRoot().boundingBox);

        for (STRTreeNode s : strTree.getRoot().children) {
            System.out.println("--> " + s.boundingBox);

            for (STRTreeNode t : s.children) {
                System.out.println("----> " + t.boundingBox);

                for (STRTreeNode b : t.children) {
                    System.out.println("------>" + b.boundingBox);
                }
            }
        }
    }

    public int getLeafCapacity() {
        return leafCapacity;
    }

    public List<Rectangle> getBoundingBoxes() {
        return boundingBoxes;
    }

    public STRTreeNode getRoot() {
        return root;
    }

    private void addRoot(Rectangle boundingBox) {
        if (root == null) {
            root = new STRTreeNode(null, new LinkedList<>(), this, boundingBox);
        } else {
            throw new IllegalStateException("Trying to add a root node to a non-empty tree.");
        }
    }

    private STRTreeNode addNode(STRTreeNode parent, Rectangle boundingBox) {
        if (parent == null) {
            throw new NullPointerException("Cannot add child to null parent.");
        } else if (parent.hostTree != this) {
            throw new IllegalArgumentException("Parent node not a part of this tree.");
        } else {
            STRTreeNode newNode = new STRTreeNode(parent, new LinkedList<>(), this, boundingBox);
            parent.children.addLast(newNode);
            return newNode;
        }
    }

    private void calculateTree() {
        int leafLevelPages = (int) Math.ceil(boundingBoxes.size() / (double) leafCapacity);
        int numberOfVerticleSlices = (int) Math.ceil(Math.sqrt(leafLevelPages));
        int sliceCapacity = numberOfVerticleSlices * leafCapacity;
        int numberOfTilesPerSlice = (int) Math.ceil(sliceCapacity / (double) leafCapacity);

        Rectangle allEncompassing = new Rectangle();

        addRoot(allEncompassing);

        boundingBoxes.sort(xComparator);

        Iterator i = boundingBoxes.iterator();

        for (int j = 0; j < numberOfVerticleSlices; j++) {
            Rectangle currentSliceEncompassing = new Rectangle();
            List<Rectangle> currentSliceBoundingBoxes = new ArrayList<>();

            int boundingBoxesAddedToSlice = 0;

            while (i.hasNext() && boundingBoxesAddedToSlice < sliceCapacity) {
                Rectangle sliceBoundingBox = (Rectangle) i.next();

                allEncompassing.expandToInclude(sliceBoundingBox);

                currentSliceEncompassing.expandToInclude(sliceBoundingBox);
                currentSliceBoundingBoxes.add(sliceBoundingBox);
                boundingBoxesAddedToSlice++;
            }

            STRTreeNode sliceNode = addNode(root, currentSliceEncompassing);

            currentSliceBoundingBoxes.sort(yComparator);

            Iterator k = currentSliceBoundingBoxes.iterator();

            for (int l = 0; l < numberOfTilesPerSlice; l++) {
                Rectangle currentTileEncompassing = new Rectangle();
                List<Rectangle> currentTileBoundingBoxes = new ArrayList<>();

                int boundingBoxesAddedToTile = 0;

                while (k.hasNext() && boundingBoxesAddedToTile < leafCapacity) {
                    Rectangle tileBoundingBox = (Rectangle) k.next();
                    currentTileEncompassing.expandToInclude(tileBoundingBox);
                    currentTileBoundingBoxes.add(tileBoundingBox);
                    boundingBoxesAddedToTile++;
                }

                STRTreeNode tileNode = addNode(sliceNode, currentTileEncompassing);

                for (Rectangle tileBB : currentTileBoundingBoxes) {
                    addNode(tileNode, tileBB);
                }
            }
        }
    }

}
