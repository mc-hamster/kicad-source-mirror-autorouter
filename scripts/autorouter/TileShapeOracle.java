/* QA only: exercise pinned Freerouting TileShape/PolylineShape methods. */
import app.freerouting.geometry.planar.*;
import java.util.*;

public class TileShapeOracle {
  private static Line[] polygon(int[][] points) {
    Line[] result = new Line[points.length];
    for (int index = 0; index < points.length; ++index) {
      int[] a = points[index];
      int[] b = points[(index + 1) % points.length];
      result[index] = new Line(a[0], a[1], b[0], b[1]);
    }
    return result;
  }

  private static Simplex base(int kind, int dx, int dy) {
    Line[] lines = switch (kind) {
      case 0 -> polygon(new int[][]{{dx - 24, dy - 15}, {dx + 22, dy - 15},
                                     {dx + 22, dy + 17}, {dx - 24, dy + 17}});
      case 1 -> polygon(new int[][]{{dx - 25, dy - 13}, {dx + 23, dy - 8},
                                     {dx + 7, dy + 26}});
      case 2 -> polygon(new int[][]{{dx - 25, dy - 8}, {dx - 11, dy - 23},
                                     {dx + 17, dy - 16}, {dx + 26, dy + 5},
                                     {dx + 5, dy + 27}, {dx - 21, dy + 18}});
      default -> polygon(new int[][]{{dx - 20, dy - 10}, {dx - 10, dy - 20},
                                      {dx + 16, dy - 20}, {dx + 26, dy - 10},
                                      {dx + 26, dy + 13}, {dx + 14, dy + 25},
                                      {dx - 9, dy + 25}, {dx - 20, dy + 14}});
    };
    return Simplex.getInstance(lines);
  }

  private static void line(StringBuilder out, Line value) {
    IntPoint a = (IntPoint) value.a;
    IntPoint b = (IntPoint) value.b;
    out.append(a.x).append(' ').append(a.y).append(' ')
       .append(b.x).append(' ').append(b.y);
  }

  private static void simplex(StringBuilder out, Simplex value) {
    out.append(value.borderLineCount());
    for (int index = 0; index < value.borderLineCount(); ++index) {
      out.append(' ');
      line(out, value.borderLine(index));
    }
  }

  private static void floatPoint(StringBuilder out, FloatPoint value) {
    out.append(String.format(Locale.ROOT, "%.9f %.9f", value.x, value.y));
  }

  private static void floatLine(StringBuilder out, FloatLine value) {
    if (value == null) {
      out.append(-1);
    } else {
      out.append("4 ");
      floatPoint(out, value.a);
      out.append(' ');
      floatPoint(out, value.b);
    }
  }

  private static int side(Side value) {
    if (value == Side.ON_THE_LEFT) return 1;
    if (value == Side.ON_THE_RIGHT) return -1;
    return 0;
  }

  public static void main(String[] args) throws Exception {
    Random random = new Random(230108);
    double[] offsets = {0.0, 0.49, 1.0, 1.4, 2.5, -0.49, -1.0, -1.4};
    int[][] directions = {{1, 0}, {1, 1}, {0, 1}, {-2, 3}, {-1, 0},
                          {-3, -2}, {0, -1}, {4, -3}};

    for (int record = 0; record < 512; ++record) {
      int dx = random.nextInt(101) - 50;
      int dy = random.nextInt(101) - 50;
      Simplex shape = base(record % 4, dx, dy).offset(offsets[record % offsets.length]);

      Simplex other;
      if (record % 3 == 0) {
        other = base((record + 1) % 4, dx + 4, dy - 3).offset(-2.0);
      } else if (record % 3 == 1) {
        other = base(0, dx + 22, dy + 2).offset(-3.0);
      } else {
        other = base(1, dx + 90, dy + 70).offset(-2.0);
      }

      FloatPoint query = switch (record % 4) {
        case 0 -> new FloatPoint(dx + 0.25, dy - 0.75);
        case 1 -> new FloatPoint(dx + 48.125, dy + 31.5);
        case 2 -> shape.cornerApprox(record % shape.borderLineCount());
        default -> new FloatPoint(dx - 39.75, dy + 22.125);
      };
      FloatPoint centre = shape.centreOfGravity();
      IntPoint rayStart = centre.round();
      int[] direction = directions[record % directions.length];
      int count = record % 7;
      double tolerance = new double[]{0.0, 0.25, 1.0, 3.5}[record % 4];
      Line probe = new Line(dx - 55, dy + (record % 17) - 8,
                            dx + 61, dy + (record % 23) - 11);
      IntBox container = record % 2 == 0
          ? new IntBox(dx - 80, dy - 80, dx + 80, dy + 80)
          : new IntBox(dx - 12, dy - 12, dx + 12, dy + 12);

      StringBuilder out = new StringBuilder("TILE2 ");
      simplex(out, shape);
      out.append(' ');
      simplex(out, other);
      // The query may itself be a rational support intersection.  Preserve
      // enough input precision for the native side tests; result values below
      // remain normalized to nine decimals for stable cross-runtime output.
      out.append(' ').append(Double.toString(query.x)).append(' ')
         .append(Double.toString(query.y));
      out.append(' ').append(rayStart.x).append(' ').append(rayStart.y);
      out.append(' ').append(count).append(' ')
         .append(String.format(Locale.ROOT, "%.9f", tolerance));
      out.append(' ');
      line(out, probe);
      out.append(' ').append(container.ll.x).append(' ').append(container.ll.y)
         .append(' ').append(container.ur.x).append(' ').append(container.ur.y);
      out.append(' ').append(direction[0]).append(' ').append(direction[1]);

      out.append(' ').append(shape.contains(query, tolerance) ? 1 : 0);
      out.append(' ').append(side(shape.sideOfBorder(query, tolerance)));
      out.append(' ').append(shape.contains(other) ? 1 : 0);
      out.append(' ').append(shape.containsApprox(other) ? 1 : 0);
      out.append(' ').append(String.format(Locale.ROOT, "%.9f %.9f %.9f %.9f",
          shape.distance(query), shape.borderDistance(query),
          shape.smallestRadius(), shape.length()));

      FloatPoint[] nearest = shape.nearestBorderPointsApprox(query, count);
      out.append(' ').append(nearest.length);
      for (FloatPoint point : nearest) {
        out.append(' ');
        floatPoint(out, point);
      }
      out.append(' ').append(shape.indexOfNearestCorner(rayStart));
      out.append(' ');
      floatLine(out, shape.diagonalCornerSegment());

      FloatPoint[] outside = shape.nearestRelativeOutsideLocations(other, count);
      out.append(' ').append(outside.length);
      for (FloatPoint point : outside) {
        out.append(' ');
        floatPoint(out, point);
      }
      out.append(' ');
      simplex(out, (Simplex) shape.shrink(1.25));

      out.append(' ').append(shape.indexOfLeftMostCorner(query));
      out.append(' ').append(shape.indexOfRightMostCorner(query));
      out.append(' ').append(shape.indexOfRightMostCorner(rayStart));
      out.append(' ');
      floatLine(out, shape.polarLineSegment(query));
      out.append(' ').append(shape.intersects(probe) ? 1 : 0);
      out.append(' ').append(ConvexGeometryOracle.point(shape.leftMostCorner(rayStart)));
      out.append(' ').append(ConvexGeometryOracle.point(shape.rightMostCorner(rayStart)));
      out.append(' ').append(shape.isContainedIn(container) ? 1 : 0);
      out.append(' ').append(shape.intersectingBorderLineNo(
          rayStart, Direction.getInstance(new IntVector(direction[0], direction[1]))));
      System.out.println(out);
    }
  }
}
