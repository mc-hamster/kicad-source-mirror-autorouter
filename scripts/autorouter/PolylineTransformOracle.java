/* QA only: exercise pinned Freerouting Line/Polyline transformation helpers directly. */
import app.freerouting.geometry.planar.*;
import java.util.*;

public class PolylineTransformOracle {
  private static String line(Line value) {
    return ConvexGeometryOracle.line(value);
  }

  private static void lines(StringBuilder out, Polyline value) {
    out.append(value.lines.length);
    for (Line current : value.lines) out.append(' ').append(line(current));
  }

  private static void segment(StringBuilder out, LineSegment value) {
    if (value == null) {
      out.append(-1);
      return;
    }
    out.append(line(value.getStartClosingLine())).append(' ')
       .append(line(value.getLine())).append(' ')
       .append(line(value.getEndClosingLine()));
  }

  private static void simplex(StringBuilder out, TileShape value) {
    Simplex shape = value.toSimplex();
    out.append(shape.borderLineCount());
    for (int index = 0; index < shape.borderLineCount(); ++index) {
      out.append(' ').append(line(shape.borderLine(index)));
    }
  }

  private static Polyline fromPoints(int[][] coordinates) {
    Point[] points = new Point[coordinates.length];
    for (int index = 0; index < coordinates.length; ++index) {
      points[index] = new IntPoint(coordinates[index][0], coordinates[index][1]);
    }
    return new Polyline(points);
  }

  private static Polyline path(int kind, int dx, int dy) {
    return switch (kind) {
      case 0 -> fromPoints(new int[][]{{dx - 30, dy - 10}, {dx - 5, dy - 10},
                                       {dx - 5, dy + 22}, {dx + 31, dy + 22}});
      case 1 -> fromPoints(new int[][]{{dx - 30, dy - 20}, {dx - 12, dy - 2},
                                       {dx + 7, dy - 21}, {dx + 31, dy + 3}});
      case 2 -> fromPoints(new int[][]{{dx - 33, dy - 18}, {dx - 11, dy - 7},
                                       {dx + 4, dy + 19}, {dx + 28, dy + 12},
                                       {dx + 36, dy - 17}});
      case 3 -> new Polyline(new Line[]{
          new Line(dx - 29, dy - 31, dx - 24, dy - 9),
          new Line(dx - 31, dy - 14, dx + 27, dy - 3),
          new Line(dx + 18, dy - 28, dx + 25, dy + 29),
          new Line(dx + 32, dy + 14, dx - 23, dy + 25),
          new Line(dx - 19, dy + 31, dx - 17, dy + 2)});
      case 4 -> fromPoints(new int[][]{{dx - 34, dy}, {dx - 4, dy},
                                       {dx - 3, dy + 1}, {dx - 2, dy},
                                       {dx + 32, dy}});
      case 5 -> fromPoints(new int[][]{{dx - 31, dy - 17}, {dx - 13, dy + 7},
                                       {dx + 2, dy - 14}, {dx + 13, dy + 9},
                                       {dx + 34, dy - 2}});
      case 6 -> fromPoints(new int[][]{{dx - 35, dy - 8}, {dx + 29, dy + 17}});
      default -> fromPoints(new int[][]{{dx - 38, dy - 7}, {dx - 18, dy + 13},
                                         {dx - 3, dy - 2}, {dx + 12, dy + 13},
                                         {dx + 34, dy - 9}});
    };
  }

  private static double canonicalZero(double value) {
    return value == 0 ? 0 : value;
  }

  public static void main(String[] args) throws Exception {
    Random random = new Random(230107);
    for (int record = 0; record < 384; ++record) {
      int dx = random.nextInt(101) - 50;
      int dy = random.nextInt(101) - 50;
      Polyline value = path(record % 8, dx, dy);
      int factor = record % 11 - 5;
      IntPoint pole = new IntPoint(dx + record % 13 - 6, dy + record % 17 - 8);
      double angle = (record % 9 - 4) * Math.PI / 17.0;
      FloatPoint rotatePole = new FloatPoint(dx + 0.25 * (record % 5),
                                             dy - 0.375 * (record % 7));
      IntPoint probe = switch (record % 5) {
        case 0 -> new IntPoint(dx, dy);
        case 1 -> value.firstCorner().toFloat().round();
        case 2 -> value.lastCorner().toFloat().round();
        case 3 -> new IntPoint(dx + 57, dy - 43);
        default -> new IntPoint(dx - 41, dy + 52);
      };
      int newLineCount = record % 3 == 0 && value.lines.length > 3
                         ? value.lines.length - 1 : value.lines.length;
      double lastLength = 3.5 + record % 23;
      int halfWidth = record % 12;
      int segmentIndex = record % (value.lines.length - 2);
      double queryX = dx + (record % 19) * 3.25 - 29.5;
      double queryY = dy + (record % 23) * 2.75 - 31.25;
      int testedDx = 47 + record % 7;
      int testedDy = 29 + record % 5;
      Line testedLine = new Line(dx - 33, dy - 17,
                                 dx - 33 + testedDx, dy - 17 + testedDy);
      Line otherLine = switch (record % 3) {
        case 0 -> new Line(dx + 7, dy - 11, dx + 7 - testedDy, dy - 11 + testedDx);
        case 1 -> new Line(dx - 19, dy + 23, dx - 19 + testedDx, dy + 23 + testedDy);
        default -> new Line(dx + 31, dy - 27, dx - 16, dy + 14);
      };

      StringBuilder out = new StringBuilder("PTRANS ");
      lines(out, value);
      out.append(' ').append(factor).append(' ').append(pole.x).append(' ').append(pole.y)
         .append(' ').append(String.format(Locale.ROOT, "%.17g", angle))
         .append(' ').append(String.format(Locale.ROOT, "%.17g %.17g",
                                           rotatePole.x, rotatePole.y))
         .append(' ').append(probe.x).append(' ').append(probe.y)
         .append(' ').append(newLineCount)
         .append(' ').append(String.format(Locale.ROOT, "%.9f", lastLength))
         .append(' ').append(halfWidth).append(' ').append(segmentIndex)
         .append(' ').append(String.format(Locale.ROOT, "%.9f %.9f", queryX, queryY))
         .append(' ').append(line(testedLine)).append(' ').append(line(otherLine));

      out.append(' ');
      lines(out, value.turn90Degree(factor, pole));
      out.append(' ');
      lines(out, value.rotateApprox(angle, rotatePole));
      out.append(' ');
      lines(out, value.mirrorVertical(pole));
      out.append(' ');
      lines(out, value.mirrorHorizontal(pole));
      out.append(' ').append(String.format(Locale.ROOT, "%.9f",
          canonicalZero(value.distance(new FloatPoint(queryX, queryY)))));
      out.append(' ');
      segment(out, value.projectionLine(probe));
      out.append(' ');
      lines(out, value.shorten(newLineCount, lastLength));
      out.append(' ');
      simplex(out, value.offsetShape(halfWidth, segmentIndex));
      IntBox offsetBox = value.offsetBox(halfWidth, segmentIndex);
      out.append(' ').append(offsetBox.ll.x).append(' ').append(offsetBox.ll.y)
         .append(' ').append(offsetBox.ur.x).append(' ').append(offsetBox.ur.y);

      out.append(' ').append(line(testedLine.turn90Degree(factor, pole)))
         .append(' ').append(line(testedLine.mirrorVertical(pole)))
         .append(' ').append(line(testedLine.mirrorHorizontal(pole)))
         .append(' ').append(testedLine.isPerpendicular(otherLine) ? 1 : 0)
         .append(' ').append(String.format(Locale.ROOT, "%.9f %.9f %.9f %.9f",
             canonicalZero(testedLine.cosAngle(otherLine)),
             canonicalZero(testedLine.functionValueApprox(queryX)),
             canonicalZero(testedLine.functionInYValueApprox(queryY)),
             canonicalZero(testedLine.length())));
      Direction direction = testedLine.perpendicularDirection(probe);
      if (direction == null) {
        out.append(" -1");
      } else {
        IntVector vector = (IntVector) direction.getVector();
        out.append(" 2 ").append(vector.x).append(' ').append(vector.y);
      }
      System.out.println(out);
    }
  }
}
