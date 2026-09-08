/* QA-only caller of the pinned Java implementation; no copied estimate formula.
 * Source checkout and release JAR are never modified by this harness.
 */
import app.freerouting.autoroute.maze.AutorouteControl.ExpansionCostFactor;
import app.freerouting.autoroute.maze.DestinationDistance;
import app.freerouting.geometry.planar.FloatPoint;
import app.freerouting.geometry.planar.IntBox;
import java.util.ArrayList;
import java.util.Random;

public class DestinationDistanceOracle {
  record Target(IntBox box, int layer) {}

  static String box(IntBox b) {
    return b.ll.x + " " + b.ll.y + " " + b.ur.x + " " + b.ur.y;
  }

  public static void main(String[] args) {
    Random random = new Random(230092);
    for (int n : new int[] {1, 2, 3, 4, 6}) {
      for (int activeMask = 0; activeMask < (1 << n); activeMask++) {
        for (int targetMask = 0; targetMask < 8; targetMask++) {
          for (int sample = 0; sample < 2; sample++) {
            ExpansionCostFactor[] costs = new ExpansionCostFactor[n];
            boolean[] active = new boolean[n];
            double normal = sample == 0 ? 0 : 10 + random.nextInt(10000);
            double cheap = 0.8 * normal;
            for (int layer = 0; layer < n; layer++) {
              // Includes asymmetric directions and the source's unusual case
              // of non-unit preferred-direction costs on inner layers.
              costs[layer] = new ExpansionCostFactor(
                  0.5 + random.nextInt(12) * (sample == 0 ? 0.1 : 0.25), 0.5 + random.nextInt(12) * (sample == 0 ? 0.1 : 0.25));
              active[layer] = (activeMask & (1 << layer)) != 0;
            }
            DestinationDistance distance = new DestinationDistance(costs, active, normal, cheap);
            ArrayList<Target> targets = new ArrayList<>();
            for (int layer = 0; layer < n; layer++) {
              int category = layer == 0 ? 1 : layer == n - 1 ? 2 : 4;
              if ((targetMask & category) == 0) continue;
              for (int item = 0; item < 3; item++) {
                int x = random.nextInt(400001) - 200000;
                int y = random.nextInt(400001) - 200000;
                // Point, line and area boxes; disjoint boxes test union, not
                // minimum-of-individual-destination distance.
                IntBox b = new IntBox(x, y, x + (item == 0 ? 0 : random.nextInt(20000)),
                    y + (item == 2 ? random.nextInt(20000) : 0));
                targets.add(new Target(b, layer));
                distance.join(b, layer);
              }
            }
            for (int layer = 0; layer < n; layer++) {
              int x = sample == 0 ? random.nextInt(800001) - 400000 : -6000000;
              int y = sample == 0 ? random.nextInt(800001) - 400000 : 6000000;
              IntBox query = new IntBox(x, y, x + random.nextInt(1000), y + random.nextInt(1000));
              FloatPoint point = new FloatPoint(x - 0.5, y + 0.25);
              StringBuilder line = new StringBuilder("DEST " + n + " " + activeMask + " " + normal + " " + cheap);
              for (var c : costs) line.append(' ').append(c.horizontal()).append(' ').append(c.vertical());
              line.append(' ').append(targets.size());
              for (var target : targets) line.append(' ').append(target.layer()).append(' ').append(box(target.box()));
              line.append(' ').append(layer).append(' ').append(box(query))
                  .append(' ').append(point.x).append(' ').append(point.y)
                  .append(' ').append(distance.calculate(query, layer))
                  .append(' ').append(distance.calculate(point, layer))
                  .append(' ').append(distance.calculateCheapDistance(query, layer))
                  .append(' ').append(distance.calculate(query, layer));
              System.out.println(line);
            }
          }
        }
      }
    }
  }
}
