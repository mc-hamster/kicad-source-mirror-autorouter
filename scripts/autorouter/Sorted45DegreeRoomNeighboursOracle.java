/* QA only: invokes the actual pinned Sorted45DegreeRoomNeighbours internals. */
import app.freerouting.autoroute.expansion.*;
import app.freerouting.autoroute.maze.AutorouteEngine;
import app.freerouting.board.facade.BasicBoard;
import app.freerouting.board.facade.RoutingBoard;
import app.freerouting.board.model.items.Trace;
import app.freerouting.board.searchtree.SearchTreeObject;
import app.freerouting.board.trace.PolylineTrace;
import app.freerouting.geometry.planar.*;
import java.lang.reflect.*;
import java.util.*;
import sun.misc.Unsafe;

public class Sorted45DegreeRoomNeighboursOracle {
  private record Input(int id, IntOctagon shape) {}

  private static Unsafe unsafe() throws Exception {
    Field field = Unsafe.class.getDeclaredField("theUnsafe");
    field.setAccessible(true);
    return (Unsafe) field.get(null);
  }

  private static AutorouteEngine engine(IntBox bounds) throws Exception {
    Unsafe unsafe = unsafe();
    RoutingBoard board = (RoutingBoard) unsafe.allocateInstance(RoutingBoard.class);
    Field boundingBox = BasicBoard.class.getField("boundingBox");
    unsafe.putObject(board, unsafe.objectFieldOffset(boundingBox), bounds);
    AutorouteEngine engine = (AutorouteEngine) unsafe.allocateInstance(AutorouteEngine.class);
    Field boardField = AutorouteEngine.class.getField("board");
    unsafe.putObject(engine, unsafe.objectFieldOffset(boardField), board);
    return engine;
  }

  private static ObstacleExpansionRoom obstacleRoom(IntOctagon shape) throws Exception {
    Unsafe unsafe = unsafe();
    PolylineTrace trace = (PolylineTrace) unsafe.allocateInstance(PolylineTrace.class);
    Field layer = Trace.class.getDeclaredField("layer");
    unsafe.putInt(trace, unsafe.objectFieldOffset(layer), 2);
    ObstacleExpansionRoom room =
        (ObstacleExpansionRoom) unsafe.allocateInstance(ObstacleExpansionRoom.class);
    Field item = ObstacleExpansionRoom.class.getDeclaredField("item");
    Field index = ObstacleExpansionRoom.class.getDeclaredField("indexInItem");
    Field roomShape = ObstacleExpansionRoom.class.getDeclaredField("shape");
    Field doors = ObstacleExpansionRoom.class.getDeclaredField("doors");
    unsafe.putObject(room, unsafe.objectFieldOffset(item), trace);
    unsafe.putInt(room, unsafe.objectFieldOffset(index), 0);
    unsafe.putObject(room, unsafe.objectFieldOffset(roomShape), shape);
    unsafe.putObject(room, unsafe.objectFieldOffset(doors), new ArrayList<ExpansionDoor>());
    return room;
  }

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
    Method calculateGaps = klass.getDeclaredMethod("calculateNewIncompleteRooms", AutorouteEngine.class);
    calculateGaps.setAccessible(true);
    Method calculateEdgeGaps =
        klass.getDeclaredMethod(
            "calculateEdgeIncompleteRoomsOfObstacleExpansionRoom",
            int.class,
            int.class,
            AutorouteEngine.class);
    calculateEdgeGaps.setAccessible(true);
    Field incompleteRooms = AutorouteEngine.class.getDeclaredField("incompleteExpansionRooms");
    incompleteRooms.setAccessible(true);

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
      AutorouteEngine engine = engine(board.boundingBox());
      if (!sorted.isEmpty()) calculateGaps.invoke(sorter, engine);
      @SuppressWarnings("unchecked")
      List<IncompleteFreeSpaceExpansionRoom> gaps =
          (List<IncompleteFreeSpaceExpansionRoom>) incompleteRooms.get(engine);
      out.append(" GAPS ").append(gaps == null ? 0 : gaps.size());
      if (gaps != null) {
        for (IncompleteFreeSpaceExpansionRoom gap : gaps) {
          appendOctagon(out, gap.getShape().boundingOctagon());
          appendOctagon(out, gap.getContainedShape().boundingOctagon());
        }
      }
      ObstacleExpansionRoom obstacleRoom = obstacleRoom(room);
      Object obstacleSorter = constructor.newInstance(obstacleRoom, obstacleRoom);
      for (Input input : inputs) {
        IntOctagon intersection = room.intersection(input.shape());
        CompleteFreeSpaceExpansionRoom neighbour =
            new CompleteFreeSpaceExpansionRoom(input.shape(), 2, input.id());
        add.invoke(obstacleSorter, neighbour, input.shape(), intersection);
      }
      AutorouteEngine obstacleEngine = engine(board.boundingBox());
      SortedSet<?> obstacleSorted = (SortedSet<?>) sortedField.get(obstacleSorter);
      if (obstacleSorted.isEmpty()) calculateEdgeGaps.invoke(obstacleSorter, 0, 7, obstacleEngine);
      else calculateGaps.invoke(obstacleSorter, obstacleEngine);
      @SuppressWarnings("unchecked")
      List<IncompleteFreeSpaceExpansionRoom> obstacleGaps =
          (List<IncompleteFreeSpaceExpansionRoom>) incompleteRooms.get(obstacleEngine);
      out.append(" OBSTACLE_GAPS ").append(obstacleGaps == null ? 0 : obstacleGaps.size());
      if (obstacleGaps != null) {
        for (IncompleteFreeSpaceExpansionRoom gap : obstacleGaps) {
          appendOctagon(out, gap.getShape().boundingOctagon());
          appendOctagon(out, gap.getContainedShape().boundingOctagon());
        }
      }
      System.out.println(out);
    }
  }
}
