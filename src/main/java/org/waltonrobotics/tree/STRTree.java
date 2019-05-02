package org.waltonrobotics.tree;

import org.waltonrobotics.geometry.Rectangle;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.List;

public class STRTree {

    private int leafCapacity;
    private List<Rectangle> boundingBoxes;
    private STRTreeNode root;

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
        int leafLevelPages = (int)Math.ceil(boundingBoxes.size() / (double)leafCapacity);
        int numberOfVerticleSlices = (int)Math.ceil(Math.sqrt(leafLevelPages));
        int sliceCapacity = numberOfVerticleSlices * leafCapacity;
        int sliceLeftover = boundingBoxes.size() % sliceCapacity;
        int sliceLastIndex = sliceLeftover + (int)Math.floor(boundingBoxes.size() / (double)sliceCapacity) * sliceCapacity - 1;

        System.out.println(sliceLastIndex);

        Rectangle allEncompassing = new Rectangle();
        Rectangle currentSliceEncompassing = new Rectangle();

        List<Rectangle> currentSliceBoundingBoxes = new ArrayList<>();

        int boundingBoxesAddedToSlice = 0;

        addRoot(allEncompassing);

        System.out.println(boundingBoxes.size());
        System.out.println(sliceCapacity);

        boundingBoxes.sort(xComparator);

        for (int i = 0; i < boundingBoxes.size(); i++) {
            Rectangle bb = boundingBoxes.get(i);

            allEncompassing.expandToInclude(bb);

            currentSliceEncompassing.expandToInclude(bb);
            currentSliceBoundingBoxes.add(bb);
            boundingBoxesAddedToSlice++;

            if (boundingBoxesAddedToSlice >= sliceCapacity || i == sliceLastIndex) {
                Rectangle currentTileEncompassing = new Rectangle();

                List<Rectangle> currentTileBoundingBoxes = new ArrayList<>();

                int boundingBoxesAddedToTile = 0;

                STRTreeNode sliceNode = addNode(root, currentSliceEncompassing);

                currentSliceBoundingBoxes.sort(yComparator);

                for (Rectangle currentSliceBoundingBox : currentSliceBoundingBoxes) {
                    currentTileEncompassing.expandToInclude(currentSliceBoundingBox);
                    currentTileBoundingBoxes.add(currentSliceBoundingBox);
                    boundingBoxesAddedToTile++;

                    if (boundingBoxesAddedToTile >= leafCapacity) {
                        STRTreeNode tileNode = addNode(sliceNode, currentTileEncompassing);

                        for (Rectangle tileBB : currentTileBoundingBoxes) {
                            addNode(tileNode, tileBB);
                        }

                        currentTileEncompassing = new Rectangle();
                        currentTileBoundingBoxes.clear();
                        boundingBoxesAddedToTile = 0;
                    }
                }

                currentSliceEncompassing = new Rectangle();
                currentSliceBoundingBoxes.clear();
                boundingBoxesAddedToSlice = 0;
            }
        }
    }

    public static void printTree(STRTreeNode node) {
        System.out.println(node.boundingBox);

        for (STRTreeNode child : node.children) {
            printTree(child);
        }
    }

    public static void main(String[] args) {
        List<Rectangle> boundingBoxes = new ArrayList<>();

        boundingBoxes.add(new Rectangle());
        boundingBoxes.add(new Rectangle(-2, 2, -2, 2));
        boundingBoxes.add(new Rectangle(4, 5, 4, 5));
        boundingBoxes.add(new Rectangle(6, 7, 6, 7));
        boundingBoxes.add(new Rectangle(8, 9, 8, 9));
        boundingBoxes.add(new Rectangle(10, 11, 10, 11));
        boundingBoxes.add(new Rectangle(12, 13, 12, 13));

        STRTree strTree = new STRTree(2, boundingBoxes);

        printTree(strTree.getRoot());
    }

}
