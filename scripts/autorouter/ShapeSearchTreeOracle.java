/* QA only: invokes the actual pinned general ShapeSearchTree restraint. */
package app.freerouting.board.searchtree;

import app.freerouting.autoroute.expansion.IncompleteFreeSpaceExpansionRoom;
import app.freerouting.geometry.planar.FortyfiveDegreeBoundingDirections;
import app.freerouting.geometry.planar.IntPoint;
import app.freerouting.geometry.planar.Line;
import app.freerouting.geometry.planar.Point;
import app.freerouting.geometry.planar.TileShape;
import java.lang.reflect.Method;
import java.util.Collection;
import java.util.Random;

public class ShapeSearchTreeOracle {
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

  @SuppressWarnings("unchecked")
  public static void main(String[] args) throws Exception {
    ShapeSearchTree tree =
        new ShapeSearchTree(FortyfiveDegreeBoundingDirections.INSTANCE, null, 0);
    Method restrain =
        ShapeSearchTree.class.getDeclaredMethod(
            "restrainShape", IncompleteFreeSpaceExpansionRoom.class, TileShape.class);
    restrain.setAccessible(true);

    Random random = new Random(731230100L);
    for (int test = 0; test < 2048; ++test) {
      int centerX = random.nextInt(161) - 80;
      int centerY = random.nextInt(161) - 80;
      int ux = 35 + random.nextInt(66);
      int uy = 3 + random.nextInt(13);
      int vx = -(3 + random.nextInt(13));
      int vy = 30 + random.nextInt(61);
      TileShape room = parallelogram(centerX, centerY, ux, uy, vx, vy);

      TileShape contained =
          parallelogram(
              centerX,
              centerY,
              Math.max(2, ux / 5),
              Math.max(1, uy / 5),
              Math.min(-1, vx / 5),
              Math.max(2, vy / 5));

      int obstacleX = centerX - ux + random.nextInt(2 * ux + 1);
      int obstacleY = centerY - vy + random.nextInt(2 * vy + 1);
      int obstacleUx = 10 + random.nextInt(31);
      int obstacleUy = 2 + random.nextInt(9);
      int obstacleVx = -(2 + random.nextInt(9));
      int obstacleVy = 9 + random.nextInt(32);
      TileShape obstacle =
          parallelogram(
              obstacleX,
              obstacleY,
              obstacleUx,
              obstacleUy,
              obstacleVx,
              obstacleVy);

      IncompleteFreeSpaceExpansionRoom input =
          new IncompleteFreeSpaceExpansionRoom(room, 4, contained);
      Collection<IncompleteFreeSpaceExpansionRoom> result =
          (Collection<IncompleteFreeSpaceExpansionRoom>) restrain.invoke(tree, input, obstacle);

      StringBuilder out = new StringBuilder("ROOMGENERAL");
      appendShape(out, room);
      appendShape(out, contained);
      appendShape(out, obstacle);
      out.append(' ').append(result.size());
      for (IncompleteFreeSpaceExpansionRoom candidate : result) {
        appendShape(out, candidate.getShape());
        appendShape(out, candidate.getContainedShape());
      }
      System.out.println(out);
    }
  }
}
