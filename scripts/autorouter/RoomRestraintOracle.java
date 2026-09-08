/* QA oracle only. Requires the pinned Freerouting executable JAR (GPL-3.0).
 * No Java runtime is used by the native editor.
 */
import app.freerouting.autoroute.expansion.IncompleteFreeSpaceExpansionRoom;
import app.freerouting.board.searchtree.ShapeSearchTree90Degree;
import app.freerouting.geometry.planar.IntBox;
import java.lang.reflect.Method;
import java.util.Collection;
import java.util.Random;

public class RoomRestraintOracle {
  private static String box(IntBox b) {
    return b.ll.x + " " + b.ll.y + " " + b.ur.x + " " + b.ur.y;
  }

  public static void main(String[] args) throws Exception {
    ShapeSearchTree90Degree tree = new ShapeSearchTree90Degree(null, 0);
    Method restrain = ShapeSearchTree90Degree.class.getDeclaredMethod(
        "restrainShape", IncompleteFreeSpaceExpansionRoom.class, IntBox.class);
    restrain.setAccessible(true);
    Random random = new Random(2300);
    for (int index = 0; index < 512; ++index) {
      IntBox room = new IntBox(-100, -100, 100, 100);
      int x = random.nextInt(181) - 90;
      int y = random.nextInt(181) - 90;
      IntBox contained = new IntBox(x, y, x + random.nextInt(101 - x), y + random.nextInt(101 - y));
      int ox = random.nextInt(201) - 100;
      int oy = random.nextInt(201) - 100;
      IntBox obstacle = new IntBox(ox, oy, ox + 1 + random.nextInt(150), oy + 1 + random.nextInt(150));
      @SuppressWarnings("unchecked")
      Collection<IncompleteFreeSpaceExpansionRoom> result =
          (Collection<IncompleteFreeSpaceExpansionRoom>) restrain.invoke(
              tree, new IncompleteFreeSpaceExpansionRoom(room, 2, contained), obstacle);
      StringBuilder line = new StringBuilder("CASE ").append(box(room)).append(' ')
          .append(box(contained)).append(' ').append(box(obstacle)).append(' ').append(result.size());
      for (var output : result) {
        line.append(' ').append(box(output.getShape().boundingBox())).append(' ')
            .append(box(output.getContainedShape().boundingBox()));
      }
      System.out.println(line);
    }
  }
}
