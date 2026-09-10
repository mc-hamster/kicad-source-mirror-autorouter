/* QA only: exercise pinned PolylineShape transforms and exact queries. */
import app.freerouting.geometry.planar.*;
import java.util.*;

public class TileTransformOracle {
  private static Line[] polygon(int[][] points) {
    Line[] result = new Line[points.length];
    for (int index = 0; index < points.length; ++index) {
      int[] a = points[index];
      int[] b = points[(index + 1) % points.length];
      result[index] = new Line(a[0], a[1], b[0], b[1]);
    }
    return result;
  }

  private static Simplex shape(int kind, int dx, int dy, double offset) {
    Line[] lines = switch (kind) {
      case 0 -> polygon(new int[][]{{dx - 21, dy - 14}, {dx + 24, dy - 14},
                                     {dx + 24, dy + 18}, {dx - 21, dy + 18}});
      case 1 -> polygon(new int[][]{{dx - 24, dy - 13}, {dx + 25, dy - 7},
                                     {dx + 8, dy + 27}});
      case 2 -> polygon(new int[][]{{dx - 26, dy - 6}, {dx - 10, dy - 24},
                                     {dx + 18, dy - 15}, {dx + 27, dy + 8},
                                     {dx + 4, dy + 28}, {dx - 23, dy + 16}});
      default -> polygon(new int[][]{{dx - 22, dy - 11}, {dx - 11, dy - 22},
                                      {dx + 17, dy - 22}, {dx + 28, dy - 11},
                                      {dx + 28, dy + 14}, {dx + 15, dy + 27},
                                      {dx - 9, dy + 27}, {dx - 22, dy + 14}});
    };
    return Simplex.getInstance(lines).offset(offset);
  }

  private static void line(StringBuilder out, Line value) {
    IntPoint a = (IntPoint) value.a;
    IntPoint b = (IntPoint) value.b;
    out.append(a.x).append(' ').append(a.y).append(' ')
       .append(b.x).append(' ').append(b.y);
  }

  private static void simplex(StringBuilder out, TileShape value) {
    Simplex simplex = value.toSimplex();
    out.append(simplex.borderLineCount());
    for (int index = 0; index < simplex.borderLineCount(); ++index) {
      out.append(' ');
      line(out, simplex.borderLine(index));
    }
  }

  private static void point(StringBuilder out, Point value) throws Exception {
    if (value == null) out.append(-1);
    else out.append("3 ").append(ConvexGeometryOracle.point(value));
  }

  private static void floatPoint(StringBuilder out, FloatPoint value) {
    out.append(String.format(Locale.ROOT, "%.9f %.9f", value.x, value.y));
  }

  public static void main(String[] args) throws Exception {
    Random random = new Random(230109);
    double[] offsets = {0.0, 0.49, 1.0, 1.4, 2.5, -0.49, -1.0, -1.4};
    double[] angles = {0.0, Math.PI / 17, -Math.PI / 9, Math.PI / 4,
                       Math.PI / 2, 2.3, -4.75};

    for (int record = 0; record < 384; ++record) {
      int dx = random.nextInt(121) - 60;
      int dy = random.nextInt(121) - 60;
      Simplex shape = shape(record % 4, dx, dy, offsets[record % offsets.length]);
      Point query = switch (record % 4) {
        case 0 -> new IntPoint(dx, dy);
        case 1 -> new IntPoint(dx + 58, dy + 43);
        case 2 -> shape.corner(record % shape.borderLineCount());
        default -> new IntPoint(dx - 47, dy + 31);
      };
      int factor = record % 15 - 7;
      double angle = angles[record % angles.length];
      IntPoint pole = new IntPoint(dx + (record % 9) - 4,
                                   dy + (record % 11) - 5);
      FloatPoint floatPole = new FloatPoint(pole.x + 0.25, pole.y - 0.375);

      StringBuilder out = new StringBuilder("TILEX ");
      simplex(out, shape);
      out.append(' ').append(ConvexGeometryOracle.point(query))
         .append(' ').append(factor).append(' ')
         .append(Double.toString(angle)).append(' ')
         .append(pole.x).append(' ').append(pole.y).append(' ')
         .append(Double.toString(floatPole.x)).append(' ')
         .append(Double.toString(floatPole.y));

      Point[] bounded = shape.boundedCorners();
      out.append(' ').append(bounded.length);
      for (Point value : bounded) {
        out.append(' ').append(ConvexGeometryOracle.point(value));
      }
      FloatPoint[] approximate = shape.cornerApproxArr();
      out.append(' ').append(approximate.length);
      for (FloatPoint value : approximate) {
        out.append(' ');
        floatPoint(out, value);
      }
      IntOctagon octagon = shape.boundingOctagon();
      out.append(' ').append(octagon.leftX).append(' ').append(octagon.bottomY)
         .append(' ').append(octagon.rightX).append(' ').append(octagon.topY)
         .append(' ').append(octagon.upperLeftDiagonalX)
         .append(' ').append(octagon.lowerRightDiagonalX)
         .append(' ').append(octagon.lowerLeftDiagonalX)
         .append(' ').append(octagon.upperRightDiagonalX);
      out.append(' ');
      point(out, shape.nearestPoint(query));
      out.append(' ');
      point(out, shape.nearestBorderPoint(query));
      out.append(' ').append(shape.isOutside(query) ? 1 : 0)
         .append(' ').append(shape.containsOnBorder(shape.corner(0)) ? 1 : 0);
      out.append(' ');
      simplex(out, shape.turn90Degree(factor, pole));
      out.append(' ');
      simplex(out, shape.rotateApprox(angle, floatPole));
      out.append(' ');
      simplex(out, shape.mirrorVertical(pole));
      out.append(' ');
      simplex(out, shape.mirrorHorizontal(pole));
      System.out.println(out);
    }
  }
}
