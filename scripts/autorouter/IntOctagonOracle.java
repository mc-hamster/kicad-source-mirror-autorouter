/* QA only: records produced by the actual pinned Freerouting IntOctagon. */
import app.freerouting.geometry.planar.*;
import java.util.Random;

public class IntOctagonOracle {
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

  private static void appendCutout(StringBuilder out, TileShape[] pieces) {
    out.append(' ').append(pieces.length);
    for (TileShape piece : pieces) appendOctagon(out, piece.boundingOctagon());
  }

  private static IntOctagon randomOctagon(Random random) {
    int left = random.nextInt(241) - 120;
    int bottom = random.nextInt(241) - 120;
    int right = left + random.nextInt(101) - 12;
    int top = bottom + random.nextInt(101) - 12;
    return new IntOctagon(
        left,
        bottom,
        right,
        top,
        left - top + random.nextInt(51) - 25,
        right - bottom + random.nextInt(51) - 25,
        left + bottom + random.nextInt(51) - 25,
        right + top + random.nextInt(51) - 25);
  }

  private static int sideCode(Side side) {
    if (side == Side.ON_THE_LEFT) return -1;
    if (side == Side.ON_THE_RIGHT) return 1;
    return 0;
  }

  public static void main(String[] args) {
    Random random = new Random(450230100L);
    for (int test = 0; test < 2048; ++test) {
      IntOctagon raw = randomOctagon(random);
      IntOctagon otherRaw = randomOctagon(random);
      IntOctagon octagon = raw.normalize();
      IntOctagon other = otherRaw.normalize();
      int pointX = random.nextInt(321) - 160;
      int pointY = random.nextInt(321) - 160;
      int probeX = random.nextInt(321) - 160;
      int probeY = random.nextInt(321) - 160;
      double distance = (random.nextInt(41) - 20) * 0.5;

      StringBuilder out = new StringBuilder("OCTAGON");
      appendOctagon(out, raw);
      appendOctagon(out, otherRaw);
      out.append(' ').append(pointX).append(' ').append(pointY)
          .append(' ').append(probeX).append(' ').append(probeY)
          .append(' ').append(Double.toHexString(distance));

      appendOctagon(out, octagon);
      out.append(' ').append(octagon.dimension())
          .append(' ').append(octagon.isNormalized() ? 1 : 0)
          .append(' ').append(Double.toHexString(octagon.area()));
      if (!octagon.isEmpty()) {
        for (int corner = 0; corner < 8; ++corner) {
          out.append(' ').append(octagon.cornerX(corner))
              .append(' ').append(octagon.cornerY(corner));
        }
      }

      appendOctagon(out, octagon.offset(distance));
      appendOctagon(out, octagon.union(other));
      appendOctagon(out, octagon.intersection(other));
      out.append(' ').append(octagon.isContainedIn(other) ? 1 : 0)
          .append(' ').append(octagon.intersects(other) ? 1 : 0)
          .append(' ').append(octagon.overlaps(other) ? 1 : 0)
          .append(' ').append(octagon.contains(new IntPoint(pointX, pointY)) ? 1 : 0)
          .append(' ').append(octagon.leftXValue(probeY))
          .append(' ').append(octagon.rightXValue(probeY))
          .append(' ').append(octagon.lowerYValue(probeX))
          .append(' ').append(octagon.upperYValue(probeX));
      for (int border = 0; border < 8; ++border) {
        out.append(' ').append(sideCode(octagon.sideOfBorderLine(pointX, pointY, border)))
            .append(' ').append(sideCode(octagon.compare(other, border)));
      }
      out.append(' ').append(octagon.isIntBox() ? 1 : 0);
      if (octagon.dimension() == 2 && other.dimension() == 2) {
        appendCutout(out, octagon.cutout(other));
        appendCutout(out, octagon.boundingBox().cutout(other));
      } else {
        out.append(" 0 0");
      }
      System.out.println(out);
    }
  }
}
