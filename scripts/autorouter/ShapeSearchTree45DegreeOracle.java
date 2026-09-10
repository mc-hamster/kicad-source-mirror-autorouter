/* QA only: invokes the actual pinned ShapeSearchTree45Degree restraint. */
import app.freerouting.autoroute.expansion.IncompleteFreeSpaceExpansionRoom;
import app.freerouting.board.searchtree.ShapeSearchTree45Degree;
import app.freerouting.geometry.planar.IntBox;
import app.freerouting.geometry.planar.IntOctagon;
import java.lang.reflect.Method;
import java.util.Collection;
import java.util.Random;

public class ShapeSearchTree45DegreeOracle {
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
      int left, int bottom, int right, int top, Random random, int maxCut) {
    return new IntOctagon(
            left,
            bottom,
            right,
            top,
            left - top + random.nextInt(maxCut + 1),
            right - bottom - random.nextInt(maxCut + 1),
            left + bottom + random.nextInt(maxCut + 1),
            right + top - random.nextInt(maxCut + 1))
        .normalize();
  }

  @SuppressWarnings("unchecked")
  public static void main(String[] args) throws Exception {
    ShapeSearchTree45Degree tree = new ShapeSearchTree45Degree(null, 0);
    Method restrain =
        ShapeSearchTree45Degree.class.getDeclaredMethod(
            "restrainShape", IncompleteFreeSpaceExpansionRoom.class, IntOctagon.class);
    restrain.setAccessible(true);

    Random random = new Random(451230100L);
    for (int test = 0; test < 2048; ++test) {
      int left = random.nextInt(121) - 100;
      int bottom = random.nextInt(121) - 100;
      int width = 40 + random.nextInt(121);
      int height = 40 + random.nextInt(121);
      IntOctagon room = clippedBox(left, bottom, left + width, bottom + height, random, 25);

      int centreX = (room.leftX + room.rightX) / 2;
      int centreY = (room.bottomY + room.topY) / 2;
      int halfWidth = random.nextInt(16);
      int halfHeight = random.nextInt(16);
      IntOctagon contained =
          room.intersection(
              new IntBox(
                      centreX - halfWidth,
                      centreY - halfHeight,
                      centreX + halfWidth,
                      centreY + halfHeight)
                  .toIntOctagon());

      int obstacleLeft = left - 35 + random.nextInt(width + 71);
      int obstacleBottom = bottom - 35 + random.nextInt(height + 71);
      int obstacleWidth = 10 + random.nextInt(71);
      int obstacleHeight = 10 + random.nextInt(71);
      IntOctagon obstacle =
          clippedBox(
              obstacleLeft,
              obstacleBottom,
              obstacleLeft + obstacleWidth,
              obstacleBottom + obstacleHeight,
              random,
              18);

      IncompleteFreeSpaceExpansionRoom input =
          new IncompleteFreeSpaceExpansionRoom(room, 3, contained);
      Collection<IncompleteFreeSpaceExpansionRoom> result =
          (Collection<IncompleteFreeSpaceExpansionRoom>) restrain.invoke(tree, input, obstacle);

      StringBuilder out = new StringBuilder("ROOM45");
      appendOctagon(out, room);
      appendOctagon(out, contained);
      appendOctagon(out, obstacle);
      out.append(' ').append(result.size());
      for (IncompleteFreeSpaceExpansionRoom candidate : result) {
        appendOctagon(out, candidate.getShape().boundingOctagon());
        appendOctagon(out, candidate.getContainedShape().boundingOctagon());
      }
      System.out.println(out);
    }
  }
}
