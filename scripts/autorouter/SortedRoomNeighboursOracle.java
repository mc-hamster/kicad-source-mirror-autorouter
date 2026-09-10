/* QA only: invokes the actual pinned unrestricted-angle SortedRoomNeighbours. */
import app.freerouting.autoroute.expansion.*;
import app.freerouting.autoroute.maze.AutorouteEngine;
import app.freerouting.board.model.structure.FixedState;
import app.freerouting.board.searchtree.SearchTreeObject;
import app.freerouting.board.trace.PolylineTrace;
import app.freerouting.geometry.planar.*;
import java.lang.reflect.*;
import java.util.*;
import sun.misc.Unsafe;

public class SortedRoomNeighboursOracle {
  private record Input(int id, Simplex shape) {}

  private static Unsafe unsafe() throws Exception {
    Field field = Unsafe.class.getDeclaredField("theUnsafe");
    field.setAccessible(true);
    return (Unsafe) field.get(null);
  }

  private static AutorouteEngine engine() throws Exception {
    return (AutorouteEngine) unsafe().allocateInstance(AutorouteEngine.class);
  }

  private static ObstacleExpansionRoom obstacleRoom(TileShape shape, int id) throws Exception {
    Point[] points = {new IntPoint(-10, 0), new IntPoint(10, 0)};
    PolylineTrace trace =
        new PolylineTrace(
            new Polyline(points), 2, 1, new int[] {1}, 0, id, 0,
            FixedState.UNFIXED, null);
    Unsafe unsafe = unsafe();
    ObstacleExpansionRoom room =
        (ObstacleExpansionRoom) unsafe.allocateInstance(ObstacleExpansionRoom.class);
    Field item = ObstacleExpansionRoom.class.getDeclaredField("item");
    Field index = ObstacleExpansionRoom.class.getDeclaredField("indexInItem");
    Field roomShape = ObstacleExpansionRoom.class.getDeclaredField("shape");
    Field doors = ObstacleExpansionRoom.class.getDeclaredField("doors");
    unsafe.putObject(room, unsafe.objectFieldOffset(item), trace);
    // A non-terminal trace-shape index makes every empty-side door admissible.
    unsafe.putInt(room, unsafe.objectFieldOffset(index), 2);
    unsafe.putObject(room, unsafe.objectFieldOffset(roomShape), shape);
    unsafe.putObject(room, unsafe.objectFieldOffset(doors), new ArrayList<ExpansionDoor>());
    return room;
  }

  private static Simplex parallelogram(
      int centerX, int centerY, int ux, int uy, int vx, int vy) {
    Point[] corners = {
      new IntPoint(centerX - ux - vx, centerY - uy - vy),
      new IntPoint(centerX + ux - vx, centerY + uy - vy),
      new IntPoint(centerX + ux + vx, centerY + uy + vy),
      new IntPoint(centerX - ux + vx, centerY - uy + vy)
    };
    return TileShape.getInstance(corners).toSimplex();
  }

  private static void appendShape(StringBuilder out, TileShape shape) {
    out.append(' ').append(shape.borderLineCount());
    for (int i = 0; i < shape.borderLineCount(); ++i) {
      Line line = shape.borderLine(i);
      IntPoint a = (IntPoint) line.a;
      IntPoint b = (IntPoint) line.b;
      out.append(' ').append(a.x).append(' ').append(a.y)
          .append(' ').append(b.x).append(' ').append(b.y);
    }
  }

  private static TileShape intersectAsTile(TileShape first, TileShape second) {
    // Preserve calculateNeighbours' TileShape/TileShape overload and its
    // observable double-dispatch support order.
    return first.intersection(second);
  }

  @SuppressWarnings("unchecked")
  public static void main(String[] args) throws Exception {
    Class<?> klass = SortedRoomNeighbours.class;
    Constructor<?> constructor =
        klass.getDeclaredConstructor(ExpansionRoom.class, CompleteExpansionRoom.class);
    constructor.setAccessible(true);
    Method add =
        klass.getDeclaredMethod(
            "addSortedNeighbour", SearchTreeObject.class, TileShape.class, TileShape.class,
            int.class, int.class, boolean.class, boolean.class);
    add.setAccessible(true);
    Method calculateGaps =
        klass.getDeclaredMethod("calculateNewIncompleteRooms", AutorouteEngine.class);
    calculateGaps.setAccessible(true);
    Method calculateEmpty =
        klass.getDeclaredMethod(
            "calculateIncompleteRoomsWithEmptyNeighbours",
            ObstacleExpansionRoom.class, AutorouteEngine.class);
    calculateEmpty.setAccessible(true);
    Field sortedField = klass.getDeclaredField("sortedNeighbours");
    sortedField.setAccessible(true);
    Field incompleteRooms = AutorouteEngine.class.getDeclaredField("incompleteExpansionRooms");
    incompleteRooms.setAccessible(true);

    Random random = new Random(731240100L);
    for (int test = 0; test < 2048; ++test) {
      int centerX = random.nextInt(161) - 80;
      int centerY = random.nextInt(161) - 80;
      int ux = 30 + random.nextInt(51);
      int uy = 3 + random.nextInt(11);
      int vx = -(3 + random.nextInt(11));
      int vy = 25 + random.nextInt(46);
      Simplex room = parallelogram(centerX, centerY, ux, uy, vx, vy);
      Simplex contained =
          parallelogram(
              centerX, centerY, Math.max(2, ux / 5), Math.max(1, uy / 5),
              Math.min(-1, vx / 5), Math.max(2, vy / 5));

      int[][] shifts = {
        {2 * ux - vx, 2 * uy - vy}, {2 * ux, 2 * uy}, {2 * ux + vx, 2 * uy + vy},
        {-2 * ux - vx, -2 * uy - vy}, {-2 * ux, -2 * uy},
        {-2 * ux + vx, -2 * uy + vy},
        {2 * vx - ux, 2 * vy - uy}, {2 * vx, 2 * vy}, {2 * vx + ux, 2 * vy + uy},
        {-2 * vx - ux, -2 * vy - uy}, {-2 * vx, -2 * vy},
        {-2 * vx + ux, -2 * vy + uy},
        {2 * ux + 2 * vx, 2 * uy + 2 * vy},
        {2 * ux - 2 * vx, 2 * uy - 2 * vy},
        {-2 * ux + 2 * vx, -2 * uy + 2 * vy},
        {-2 * ux - 2 * vx, -2 * uy - 2 * vy}
      };

      ArrayList<Input> inputs = new ArrayList<>();
      if (test % 17 != 0) {
        for (int index = 0; index < shifts.length; ++index) {
          if (!random.nextBoolean()) continue;
          Simplex shape = room.translateBy(new IntVector(shifts[index][0], shifts[index][1]));
          TileShape intersection = intersectAsTile(room, shape);
          if (intersection.dimension() < 0 || intersection.dimension() > 1) continue;
          inputs.add(new Input(1000 + index, shape));
        }
      }
      Collections.shuffle(inputs, random);

      IncompleteFreeSpaceExpansionRoom from =
          new IncompleteFreeSpaceExpansionRoom(room, 2, contained);
      CompleteFreeSpaceExpansionRoom completed =
          new CompleteFreeSpaceExpansionRoom(room, 2, 900000 + test);
      Object sorter = constructor.newInstance(from, completed);
      for (Input input : inputs) {
        TileShape intersection = intersectAsTile(room, input.shape());
        int[] touchingSides = room.touchingSides(input.shape());
        int roomSide;
        int neighbourSide;
        boolean roomCorner;
        boolean neighbourCorner;
        if (intersection.dimension() == 1) {
          if (touchingSides.length != 2) continue;
          roomSide = touchingSides[0];
          neighbourSide = touchingSides[1];
          roomCorner = false;
          neighbourCorner = false;
        } else {
          Point point = intersection.corner(0);
          roomSide = room.equalsCorner(point);
          roomCorner = roomSide >= 0;
          if (!roomCorner) roomSide = room.containsOnBorderLineNo(point);
          neighbourSide = input.shape().equalsCorner(point);
          neighbourCorner = neighbourSide >= 0;
          if (neighbourCorner) neighbourSide = input.shape().prevNo(neighbourSide);
          else neighbourSide = input.shape().containsOnBorderLineNo(point);
        }
        CompleteFreeSpaceExpansionRoom neighbour =
            new CompleteFreeSpaceExpansionRoom(input.shape(), 2, input.id());
        add.invoke(
            sorter, neighbour, input.shape(), intersection, roomSide, neighbourSide,
            roomCorner, neighbourCorner);
      }

      StringBuilder out = new StringBuilder("NEIGHBOURSGENERAL");
      appendShape(out, room);
      appendShape(out, contained);
      out.append(' ').append(inputs.size());
      for (Input input : inputs) {
        out.append(' ').append(input.id());
        appendShape(out, input.shape());
      }

      SortedSet<?> sorted = (SortedSet<?>) sortedField.get(sorter);
      out.append(' ').append(sorted.size());
      int previousEdge = -1;
      int currentEdge = 0;
      for (Object neighbour : sorted) {
        Class<?> neighbourClass = neighbour.getClass();
        Field objectField = neighbourClass.getField("searchTreeObject");
        Field intersectionField = neighbourClass.getField("intersection");
        Field roomSideField = neighbourClass.getField("touchingSideNoOfRoom");
        Field neighbourSideField =
            neighbourClass.getField("touchingSideNoOfNeighbourRoom");
        Field roomCornerField = neighbourClass.getField("roomTouchIsCorner");
        Field neighbourCornerField =
            neighbourClass.getField("neighbourRoomTouchIsCorner");
        objectField.setAccessible(true);
        intersectionField.setAccessible(true);
        roomSideField.setAccessible(true);
        neighbourSideField.setAccessible(true);
        roomCornerField.setAccessible(true);
        neighbourCornerField.setAccessible(true);
        SearchTreeObject object = (SearchTreeObject) objectField.get(neighbour);
        TileShape intersection = (TileShape) intersectionField.get(neighbour);
        int roomSide = roomSideField.getInt(neighbour);
        out.append(' ').append(object.getId());
        appendShape(out, intersection);
        out.append(' ').append(roomSide)
            .append(' ').append(neighbourSideField.getInt(neighbour))
            .append(' ').append(roomCornerField.getBoolean(neighbour) ? 1 : 0)
            .append(' ').append(neighbourCornerField.getBoolean(neighbour) ? 1 : 0);
        if (roomSide != previousEdge) {
          if (roomSide == currentEdge) {
            previousEdge = currentEdge;
            ++currentEdge;
          }
        }
      }
      int firstUnrestrained = currentEdge < room.borderLineCount() ? currentEdge : -1;
      out.append(" FIRST_UNRESTRAINED ").append(firstUnrestrained);

      AutorouteEngine freeEngine = engine();
      if (!sorted.isEmpty()) calculateGaps.invoke(sorter, freeEngine);
      appendShape(out.append(" COMPLETED"), completed.getShape());
      List<IncompleteFreeSpaceExpansionRoom> gaps =
          (List<IncompleteFreeSpaceExpansionRoom>) incompleteRooms.get(freeEngine);
      out.append(" GAPS ").append(gaps == null ? 0 : gaps.size());
      if (gaps != null) {
        for (IncompleteFreeSpaceExpansionRoom gap : gaps) {
          appendShape(out, gap.getShape());
          appendShape(out, gap.getContainedShape());
        }
      }

      ObstacleExpansionRoom obstacle = obstacleRoom(room, 950000 + test);
      Object obstacleSorter = constructor.newInstance(obstacle, obstacle);
      for (Input input : inputs) {
        TileShape intersection = intersectAsTile(room, input.shape());
        int[] touchingSides = room.touchingSides(input.shape());
        int roomSide;
        int neighbourSide;
        boolean roomCorner;
        boolean neighbourCorner;
        if (intersection.dimension() == 1) {
          if (touchingSides.length != 2) continue;
          roomSide = touchingSides[0];
          neighbourSide = touchingSides[1];
          roomCorner = false;
          neighbourCorner = false;
        } else {
          Point point = intersection.corner(0);
          roomSide = room.equalsCorner(point);
          roomCorner = roomSide >= 0;
          if (!roomCorner) roomSide = room.containsOnBorderLineNo(point);
          neighbourSide = input.shape().equalsCorner(point);
          neighbourCorner = neighbourSide >= 0;
          if (neighbourCorner) neighbourSide = input.shape().prevNo(neighbourSide);
          else neighbourSide = input.shape().containsOnBorderLineNo(point);
        }
        CompleteFreeSpaceExpansionRoom neighbour =
            new CompleteFreeSpaceExpansionRoom(input.shape(), 2, input.id());
        add.invoke(
            obstacleSorter, neighbour, input.shape(), intersection, roomSide,
            neighbourSide, roomCorner, neighbourCorner);
      }
      AutorouteEngine obstacleEngine = engine();
      SortedSet<?> obstacleSorted = (SortedSet<?>) sortedField.get(obstacleSorter);
      if (obstacleSorted.isEmpty()) calculateEmpty.invoke(null, obstacle, obstacleEngine);
      else calculateGaps.invoke(obstacleSorter, obstacleEngine);
      List<IncompleteFreeSpaceExpansionRoom> obstacleGaps =
          (List<IncompleteFreeSpaceExpansionRoom>) incompleteRooms.get(obstacleEngine);
      out.append(" OBSTACLE_GAPS ").append(obstacleGaps == null ? 0 : obstacleGaps.size());
      if (obstacleGaps != null) {
        for (IncompleteFreeSpaceExpansionRoom gap : obstacleGaps) {
          appendShape(out, gap.getShape());
          appendShape(out, gap.getContainedShape());
        }
      }
      System.out.println(out);
    }
  }
}
