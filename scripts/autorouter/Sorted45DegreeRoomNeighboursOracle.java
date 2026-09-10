/* QA only: invokes the actual pinned Sorted45DegreeRoomNeighbours internals. */
import app.freerouting.autoroute.expansion.*;
import app.freerouting.board.searchtree.SearchTreeObject;
import app.freerouting.geometry.planar.*;
import java.lang.reflect.*;
import java.util.*;

public class Sorted45DegreeRoomNeighboursOracle {
  private record Input(int id, IntOctagon shape) {}

  private static void appendOctagon(StringBuilder out, IntOctagon octagon) {
    out.append(' ').append(octagon.leftX)
        .append(' ').append(octagon.bottomY)
        .append(' ').append(octagon.rightX)
        .append(' ').append(octagon.topY)
        .append(' ').append(octagon.upperLeftDiagonalX)
        .append(' ').append(octagon.lowerRightDiagonalX)
        .append(' ').append(octagon.lowerLeftDiagonalX)
        .append(' ').append(octagon.upperRightDiagonalX);
  }

  private static IntOctagon clippedBox(
      int left, int bottom, int right, int top, Random random) {
    return new IntOctagon(
            left, bottom, right, top,
            left - top + random.nextInt(21),
            right - bottom - random.nextInt(21),
            left + bottom + random.nextInt(21),
            right + top - random.nextInt(21))
        .normalize();
  }

  private static IntOctagon outside(IntOctagon room, IntOctagon board, int side) {
    int left = board.leftX, bottom = board.bottomY, right = board.rightX, top = board.topY;
    int upperLeft = board.upperLeftDiagonalX, lowerRight = board.lowerRightDiagonalX;
    int lowerLeft = board.lowerLeftDiagonalX, upperRight = board.upperRightDiagonalX;
    switch (side) {
      case 0 -> top = room.bottomY;
      case 1 -> upperLeft = room.lowerRightDiagonalX;
      case 2 -> left = room.rightX;
      case 3 -> lowerLeft = room.upperRightDiagonalX;
      case 4 -> bottom = room.topY;
      case 5 -> lowerRight = room.upperLeftDiagonalX;
      case 6 -> right = room.leftX;
      case 7 -> upperRight = room.lowerLeftDiagonalX;
      default -> throw new IllegalArgumentException();
    }
    return new IntOctagon(
            left, bottom, right, top, upperLeft, lowerRight, lowerLeft, upperRight)
        .normalize();
  }

  public static void main(String[] args) throws Exception {
    Class<?> klass = Sorted45DegreeRoomNeighbours.class;
    Constructor<?> constructor =
        klass.getDeclaredConstructor(ExpansionRoom.class, CompleteExpansionRoom.class);
    constructor.setAccessible(true);
    Method add =
        klass.getDeclaredMethod(
            "addSortedNeighbour", SearchTreeObject.class, IntOctagon.class, IntOctagon.class);
    add.setAccessible(true);
    Method remove =
        klass.getDeclaredMethod("removeNotTouchingBorderLines", IntOctagon.class, boolean[].class);
    remove.setAccessible(true);
    Field edgesField = klass.getDeclaredField("edgeInteriorTouchesObstacle");
    edgesField.setAccessible(true);
    Field sortedField = klass.getField("sortedNeighbours");

    Random random = new Random(452230100L);
    IntOctagon board = new IntBox(-300, -300, 300, 300).toIntOctagon();
    for (int test = 0; test < 2048; ++test) {
      int left = random.nextInt(101) - 100;
      int bottom = random.nextInt(101) - 100;
      IntOctagon room = clippedBox(
          left, bottom, left + 60 + random.nextInt(61), bottom + 60 + random.nextInt(61), random);
      IncompleteFreeSpaceExpansionRoom from =
          new IncompleteFreeSpaceExpansionRoom(room, 2, room);
      CompleteFreeSpaceExpansionRoom completed =
          new CompleteFreeSpaceExpansionRoom(room, 2, 900000 + test);
      Object sorter = constructor.newInstance(from, completed);

      ArrayList<Input> inputs = new ArrayList<>();
      for (int side = 0; side < 8; ++side) {
        if (!random.nextBoolean()) continue;
        int pieces = 1 + random.nextInt(2);
        for (int piece = 0; piece < pieces; ++piece) {
          IntOctagon shape = outside(room, board, side);
          int clipLeft = room.leftX - 40 + random.nextInt(61);
          int clipBottom = room.bottomY - 40 + random.nextInt(61);
          IntBox clip = new IntBox(
              clipLeft,
              clipBottom,
              clipLeft + 30 + random.nextInt(121),
              clipBottom + 30 + random.nextInt(121));
          shape = shape.intersection(clip.toIntOctagon());
          IntOctagon intersection = room.intersection(shape);
          if (shape.dimension() != 2 || intersection.dimension() < 0 || intersection.dimension() > 1)
            continue;
          int id = 1000 + side * 20 + piece;
          CompleteFreeSpaceExpansionRoom neighbour =
              new CompleteFreeSpaceExpansionRoom(shape, 2, id);
          inputs.add(new Input(id, shape));
          add.invoke(sorter, neighbour, shape, intersection);
        }
      }

      StringBuilder out = new StringBuilder("NEIGHBOURS45");
      appendOctagon(out, room);
      out.append(' ').append(inputs.size());
      for (Input input : inputs) {
        out.append(' ').append(input.id());
        appendOctagon(out, input.shape());
      }
      boolean[] edges = (boolean[]) edgesField.get(sorter);
      for (boolean edge : edges) out.append(' ').append(edge ? 1 : 0);
      appendOctagon(out, (IntOctagon) remove.invoke(null, room, edges));

      SortedSet<?> sorted = (SortedSet<?>) sortedField.get(sorter);
      out.append(' ').append(sorted.size());
      for (Object neighbour : sorted) {
        Class<?> neighbourClass = neighbour.getClass();
        Field objectField = neighbourClass.getField("searchTreeObject");
        Field intersectionField = neighbourClass.getField("intersection");
        Field firstField = neighbourClass.getField("firstTouchingSide");
        Field lastField = neighbourClass.getField("lastTouchingSide");
        objectField.setAccessible(true);
        intersectionField.setAccessible(true);
        firstField.setAccessible(true);
        lastField.setAccessible(true);
        SearchTreeObject object = (SearchTreeObject) objectField.get(neighbour);
        IntOctagon intersection = (IntOctagon) intersectionField.get(neighbour);
        int first = firstField.getInt(neighbour);
        int last = lastField.getInt(neighbour);
        out.append(' ').append(object.getId());
        appendOctagon(out, intersection);
        out.append(' ').append(first).append(' ').append(last);
      }
      System.out.println(out);
    }
  }
}
