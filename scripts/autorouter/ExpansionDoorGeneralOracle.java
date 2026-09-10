/* QA only: records produced by the actual pinned general ExpansionDoor. */
import app.freerouting.autoroute.expansion.*;
import app.freerouting.geometry.planar.*;
import java.util.Random;

public class ExpansionDoorGeneralOracle {
  private static TileShape parallelogram(
      int centerX, int centerY, int ux, int uy, int vx, int vy) {
    Point[] corners = {
      new IntPoint(centerX - ux - vx, centerY - uy - vy),
      new IntPoint(centerX + ux - vx, centerY + uy - vy),
      new IntPoint(centerX + ux + vx, centerY + uy + vy),
      new IntPoint(centerX - ux + vx, centerY - uy + vy)
    };
    return TileShape.getInstance(corners);
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

  public static void main(String[] args) {
    Random random = new Random(773230103L);
    for (int test = 0; test < 2048; ++test) {
      int centerX = random.nextInt(161) - 80;
      int centerY = random.nextInt(161) - 80;
      int ux = 25 + random.nextInt(51);
      int uy = 3 + random.nextInt(11);
      int vx = -(3 + random.nextInt(11));
      int vy = 20 + random.nextInt(46);
      TileShape first = parallelogram(centerX, centerY, ux, uy, vx, vy);
      int secondX;
      int secondY;
      switch (test % 4) {
        case 0 -> {
          secondX = centerX + ux / 2;
          secondY = centerY + uy / 2;
        }
        case 1 -> {
          secondX = centerX + 2 * ux;
          secondY = centerY + 2 * uy;
        }
        case 2 -> {
          secondX = centerX + 2 * ux + 2 * vx;
          secondY = centerY + 2 * uy + 2 * vy;
        }
        default -> {
          secondX = centerX + 4 * ux;
          secondY = centerY + 4 * uy;
        }
      }
      TileShape second = parallelogram(secondX, secondY, ux, uy, vx, vy);
      boolean completePair = random.nextBoolean();
      double offset = random.nextInt(41) * 0.25;

      ExpansionRoom firstRoom =
          completePair
              ? new CompleteFreeSpaceExpansionRoom(first, 3, 20000 + 2 * test)
              : new IncompleteFreeSpaceExpansionRoom(first, 3, first);
      ExpansionRoom secondRoom =
          new CompleteFreeSpaceExpansionRoom(second, 3, 20001 + 2 * test);
      ExpansionDoor door = new ExpansionDoor(firstRoom, secondRoom);
      FloatLine[] sections = door.getSectionSegments(offset);

      StringBuilder out = new StringBuilder("DOORGENERAL");
      appendShape(out, first);
      appendShape(out, second);
      out.append(' ').append(completePair ? 1 : 0)
          .append(' ').append(Double.toHexString(offset))
          .append(' ').append(door.dimension);
      appendShape(out, door.getShape());
      out.append(' ').append(sections.length);
      for (FloatLine section : sections) {
        out.append(' ').append(Double.toHexString(section.a.x))
            .append(' ').append(Double.toHexString(section.a.y))
            .append(' ').append(Double.toHexString(section.b.x))
            .append(' ').append(Double.toHexString(section.b.y));
      }
      System.out.println(out);
    }
  }
}
