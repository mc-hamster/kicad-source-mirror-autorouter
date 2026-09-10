/* QA only: records produced by the actual pinned Freerouting ExpansionDoor. */
import app.freerouting.autoroute.expansion.*;
import app.freerouting.geometry.planar.*;
import java.util.Random;

public class ExpansionDoorOracle {
  private static void appendOctagon(StringBuilder out, TileShape shape) {
    IntOctagon octagon = shape.boundingOctagon();
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
            left - top + random.nextInt(31),
            right - bottom - random.nextInt(31),
            left + bottom + random.nextInt(31),
            right + top - random.nextInt(31))
        .normalize();
  }

  public static void main(String[] args) {
    Random random = new Random(450230103L);
    for (int test = 0; test < 2048; ++test) {
      int left = random.nextInt(161) - 80;
      int bottom = random.nextInt(161) - 80;
      int right = left + 60 + random.nextInt(81);
      int top = bottom + 60 + random.nextInt(81);
      IntOctagon first = clippedBox(left, bottom, right, top, random);
      IntOctagon second;
      switch (test % 4) {
        case 0 -> second = clippedBox(
            left + random.nextInt(41), bottom + random.nextInt(41),
            right + random.nextInt(41), top + random.nextInt(41), random);
        case 1 -> second = new IntBox(
            first.rightX, first.bottomY + 5, first.rightX + 70,
            Math.max(first.bottomY + 6, first.topY - 5)).toIntOctagon();
        case 2 -> second = new IntBox(
            first.rightX, first.topY, first.rightX + 70, first.topY + 70).toIntOctagon();
        default -> second = clippedBox(
            right + 20 + random.nextInt(30), top + 20 + random.nextInt(30),
            right + 80 + random.nextInt(50), top + 80 + random.nextInt(50), random);
      }
      double offset = random.nextInt(41) * 0.5;
      boolean completePair = random.nextBoolean();
      ExpansionRoom firstRoom;
      ExpansionRoom secondRoom;
      if (completePair) {
        firstRoom = new CompleteFreeSpaceExpansionRoom(first, 2, 10000 + 2 * test);
        secondRoom = new CompleteFreeSpaceExpansionRoom(second, 2, 10001 + 2 * test);
      } else {
        firstRoom = new IncompleteFreeSpaceExpansionRoom(first, 2, first);
        secondRoom = new CompleteFreeSpaceExpansionRoom(second, 2, 10001 + 2 * test);
      }
      ExpansionDoor door = new ExpansionDoor(firstRoom, secondRoom);
      FloatLine[] sections = door.getSectionSegments(offset);
      StringBuilder out = new StringBuilder("DOOR45");
      appendOctagon(out, first);
      appendOctagon(out, second);
      out.append(' ').append(completePair ? 1 : 0)
          .append(' ').append(Double.toHexString(offset))
          .append(' ').append(door.dimension);
      appendOctagon(out, door.getShape());
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
