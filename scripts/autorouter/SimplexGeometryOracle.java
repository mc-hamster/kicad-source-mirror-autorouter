/* QA only: exercise the pinned Freerouting Simplex implementation directly. */
import app.freerouting.geometry.planar.*;
import java.util.*;

public class SimplexGeometryOracle {
  private static String line(Line value) {
    IntPoint a = (IntPoint) value.a;
    IntPoint b = (IntPoint) value.b;
    return a.x + " " + a.y + " " + b.x + " " + b.y;
  }

  private static void lines(StringBuilder out, Simplex value) {
    out.append(value.borderLineCount());
    for (int index = 0; index < value.borderLineCount(); ++index) {
      out.append(' ').append(line(value.borderLine(index)));
    }
  }

  private static void point(StringBuilder out, FloatPoint value) {
    out.append(String.format(Locale.ROOT, "%.9f %.9f", value.x, value.y));
  }

  private static Line[] polygon(int[][] points) {
    Line[] result = new Line[points.length];
    for (int index = 0; index < points.length; ++index) {
      int[] a = points[index];
      int[] b = points[(index + 1) % points.length];
      result[index] = new Line(a[0], a[1], b[0], b[1]);
    }
    return result;
  }

  private static Line[] input(int kind, int dx, int dy) {
    return switch (kind) {
      case 0 -> polygon(new int[][]{{-20 + dx, -12 + dy}, {19 + dx, -12 + dy},
                                     {19 + dx, 14 + dy}, {-20 + dx, 14 + dy}});
      case 1 -> polygon(new int[][]{{-23 + dx, -13 + dy}, {21 + dx, -8 + dy},
                                     {9 + dx, 24 + dy}});
      case 2 -> new Line[]{new Line(-15 + dx, -4 + dy, 17 + dx, 5 + dy)};
      case 3 -> new Line[]{new Line(-20 + dx, -9 + dy, 21 + dx, -3 + dy),
                            new Line(21 + dx, -3 + dy, 8 + dx, 22 + dy)};
      case 4 -> new Line[]{new Line(-20 + dx, -8 + dy, 20 + dx, -8 + dy),
                            new Line(20 + dx, 9 + dy, -20 + dx, 9 + dy)};
      case 5 -> new Line[]{new Line(-20 + dx, 0 + dy, 20 + dx, 0 + dy),
                            new Line(20 + dx, 0 + dy, -20 + dx, 0 + dy),
                            new Line(0 + dx, 20 + dy, 0 + dx, -20 + dy),
                            new Line(0 + dx, -20 + dy, 0 + dx, 20 + dy)};
      case 6 -> new Line[]{new Line(-20 + dx, 8 + dy, 20 + dx, 8 + dy),
                            new Line(20 + dx, -8 + dy, -20 + dx, -8 + dy)};
      default -> polygon(new int[][]{{-24 + dx, -7 + dy}, {-9 + dx, -20 + dy},
                                      {18 + dx, -14 + dy}, {25 + dx, 7 + dy},
                                      {4 + dx, 25 + dy}, {-20 + dx, 17 + dy}});
    };
  }

  private static Line[] cutoutInner(int kind, int dx, int dy) {
    return switch (kind) {
      case 0 -> polygon(new int[][]{{dx - 11, dy - 7}, {dx + 12, dy - 7},
                                     {dx + 12, dy + 8}, {dx - 11, dy + 8}});
      case 1 -> polygon(new int[][]{{dx - 13, dy - 8}, {dx + 15, dy - 3},
                                     {dx + 2, dy + 14}});
      case 2 -> polygon(new int[][]{{dx + 12, dy - 9}, {dx + 31, dy - 4},
                                     {dx + 28, dy + 16}, {dx + 9, dy + 11}});
      case 3 -> polygon(new int[][]{{dx + 45, dy + 36}, {dx + 58, dy + 36},
                                     {dx + 58, dy + 49}, {dx + 45, dy + 49}});
      case 4 -> polygon(new int[][]{{dx - 55, dy - 40}, {dx + 55, dy - 40},
                                     {dx + 55, dy + 40}, {dx - 55, dy + 40}});
      case 5 -> polygon(new int[][]{{dx - 17, dy - 2}, {dx + 14, dy - 1},
                                     {dx + 16, dy + 2}, {dx - 15, dy + 13}});
      case 6 -> new Line[]{new Line(dx - 15, dy, dx + 15, dy),
                            new Line(dx + 15, dy, dx - 15, dy)};
      default -> polygon(new int[][]{{dx - 15, dy - 4}, {dx - 5, dy - 14},
                                      {dx + 13, dy - 10}, {dx + 17, dy + 4},
                                      {dx + 3, dy + 15}, {dx - 13, dy + 10}});
    };
  }

  private static Line[] shuffled(Line[] input, Random random) {
    List<Line> result = new ArrayList<>(Arrays.asList(input));
    Collections.shuffle(result, random);
    return result.toArray(Line[]::new);
  }

  private static int side(Side value) {
    if (value == Side.ON_THE_LEFT) return 1;
    if (value == Side.ON_THE_RIGHT) return -1;
    return 0;
  }

  public static void main(String[] args) throws Exception {
    Random random = new Random(230104);
    for (int record = 0; record < 512; ++record) {
      int dx = random.nextInt(81) - 40;
      int dy = random.nextInt(81) - 40;
      Line[] source = input(record % 8, dx, dy);
      List<Line> shuffled = new ArrayList<>(Arrays.asList(source));
      if (record % 5 == 0 && source.length > 0) {
        shuffled.add(source[record % source.length]);
      }
      Collections.shuffle(shuffled, random);
      Line[] supplied = shuffled.toArray(Line[]::new);
      Simplex simplex = Simplex.getInstance(supplied);

      int removeIndex = simplex.borderLineCount() == 0 ? 0
          : record % simplex.borderLineCount();
      StringBuilder out = new StringBuilder("SIMPLEX ").append(dx).append(' ').append(dy)
          .append(' ').append(removeIndex).append(' ').append(supplied.length);
      for (Line border : supplied) {
        out.append(' ').append(line(border));
      }
      out.append(' ');
      lines(out, simplex);
      out.append(' ').append(simplex.dimension());
      out.append(' ').append(simplex.isBounded() ? 1 : 0);
      out.append(' ').append(simplex.isIntBox() ? 1 : 0);
      out.append(' ').append(simplex.isIntOctagon() ? 1 : 0);

      if (!simplex.isEmpty() && simplex.isBounded()) {
        IntBox box = simplex.boundingBox();
        out.append(" 1 ").append(box.ll.x).append(' ').append(box.ll.y)
           .append(' ').append(box.ur.x).append(' ').append(box.ur.y);
        out.append(' ').append(simplex.indexOfRightMostCorner(new IntPoint(dx + 60, dy - 55)));
        out.append(' ').append(simplex.borderLineCount());
        for (int index = 0; index < simplex.borderLineCount(); ++index) {
          out.append(' ').append(ConvexGeometryOracle.point(simplex.corner(index)));
        }
      } else {
        out.append(" 0 -1 0");
      }

      for (int[] point : new int[][]{{dx, dy}, {dx - 30, dy}, {dx + 30, dy},
                                      {dx, dy - 30}, {dx, dy + 30}}) {
        IntPoint p = new IntPoint(point[0], point[1]);
        out.append(' ').append(simplex.contains(p) ? 1 : 0);
        out.append(' ').append(simplex.containsInside(p) ? 1 : 0);
      }

      Simplex clip = new IntBox(dx - 16, dy - 11, dx + 16, dy + 11).toSimplex();
      out.append(' ');
      lines(out, simplex.intersection(clip));

      if (simplex.borderLineCount() > 0) {
        out.append(' ');
        lines(out, simplex.removeBorderLine(removeIndex));
      } else {
        out.append(" 0");
      }

      Simplex translated = simplex.translateBy(new IntVector(7, -11));
      out.append(' ');
      lines(out, translated);
      System.out.println(out);
    }

    for (int record = 0; record < 256; ++record) {
      int dx = random.nextInt(81) - 40;
      int dy = random.nextInt(81) - 40;
      Line[] outerInput = shuffled(record % 3 == 0
          ? input(7, dx, dy)
          : input(0, dx, dy), random);
      Line[] innerInput = shuffled(cutoutInner(record % 8, dx, dy), random);
      Simplex outer = Simplex.getInstance(outerInput);
      Simplex inner = Simplex.getInstance(innerInput);
      Simplex[] pieces = inner.cutoutFrom(outer);

      StringBuilder out = new StringBuilder("SCUT ").append(outerInput.length);
      for (Line border : outerInput) {
        out.append(' ').append(line(border));
      }
      out.append(' ').append(innerInput.length);
      for (Line border : innerInput) {
        out.append(' ').append(line(border));
      }
      if (pieces == null) {
        out.append(" -1");
      } else {
        out.append(' ').append(pieces.length);
        for (Simplex piece : pieces) {
          out.append(' ');
          lines(out, piece);
          out.append(' ').append(piece.dimension());
        }
      }
      System.out.println(out);
    }

    double[] widths = {0.0, 0.49, 0.5, 1.0, 1.4, 2.5, 6.25,
                       -0.49, -0.5, -1.0, -2.5, -5.5};
    for (int record = 0; record < 192; ++record) {
      int dx = random.nextInt(81) - 40;
      int dy = random.nextInt(81) - 40;
      Line[] supplied = shuffled(input(new int[]{0, 1, 7}[record % 3], dx, dy), random);
      Simplex simplex = Simplex.getInstance(supplied);
      double width = widths[record % widths.length];
      FloatPoint query = new FloatPoint(dx + (record % 9) * 9 - 36,
                                        dy + (record % 7) * 11 - 33);

      StringBuilder out = new StringBuilder("SOFF ").append(supplied.length);
      for (Line border : supplied) {
        out.append(' ').append(line(border));
      }
      out.append(' ').append(Double.toString(width));
      out.append(' ').append(String.format(Locale.ROOT, "%.9f %.9f", query.x, query.y));
      out.append(' ');
      lines(out, simplex.offset(width));
      out.append(' ');
      lines(out, simplex.enlarge(width));
      out.append(' ');
      point(out, simplex.centreOfGravity());
      out.append(' ');
      point(out, simplex.nearestPointApprox(query));
      out.append(' ');
      point(out, simplex.nearestBorderPointApprox(query));
      System.out.println(out);
    }

    for (int record = 0; record < 256; ++record) {
      int dx = random.nextInt(81) - 40;
      int dy = random.nextInt(81) - 40;
      Line[] supplied;
      Line[] otherSupplied;
      if ((record & 1) == 0) {
        supplied = polygon(new int[][]{{dx - 20, dy - 12}, {dx + 19, dy - 12},
                                        {dx + 19, dy + 14}, {dx - 20, dy + 14}});
        otherSupplied = polygon(new int[][]{{dx + 19, dy - 8}, {dx + 37, dy - 8},
                                             {dx + 37, dy + 11}, {dx + 19, dy + 11}});
      } else {
        supplied = input(record % 4 == 1 ? 1 : 7, dx, dy);
        otherSupplied = polygon(new int[][]{{dx + 70, dy - 9}, {dx + 90, dy - 9},
                                             {dx + 90, dy + 17}, {dx + 70, dy + 17}});
      }
      supplied = shuffled(supplied, random);
      otherSupplied = shuffled(otherSupplied, random);
      Simplex simplex = Simplex.getInstance(supplied);
      Simplex other = Simplex.getInstance(otherSupplied);
      Line probe = switch (record % 3) {
        case 0 -> new Line(dx - 60, dy - 20, dx + 60, dy - 20);
        case 1 -> new Line(dx + 38, dy - 60, dx + 38, dy + 60);
        default -> new Line(dx - 55, dy + 31, dx + 48, dy - 27);
      };
      IntPoint segmentStart;
      IntPoint segmentEnd;
      if (record % 3 == 0) {
        segmentStart = new IntPoint(dx - 60, dy);
        segmentEnd = new IntPoint(dx + 60, dy);
      } else if (record % 3 == 1) {
        segmentStart = new IntPoint(dx - 60, dy + 45);
        segmentEnd = new IntPoint(dx + 60, dy + 45);
      } else {
        segmentStart = new IntPoint(dx - 20, dy - 12);
        segmentEnd = new IntPoint(dx + 19, dy - 12);
      }
      Line segmentLine = new Line(segmentStart, segmentEnd);
      double sectionWidth = 9.0 + record % 11;

      StringBuilder out = new StringBuilder("STILE ").append(supplied.length);
      for (Line border : supplied) out.append(' ').append(line(border));
      out.append(' ').append(otherSupplied.length);
      for (Line border : otherSupplied) out.append(' ').append(line(border));
      out.append(' ').append(line(probe));
      out.append(' ').append(segmentStart.x).append(' ').append(segmentStart.y)
         .append(' ').append(segmentEnd.x).append(' ').append(segmentEnd.y);
      out.append(' ').append(Double.toString(sectionWidth));

      out.append(' ').append(String.format(Locale.ROOT, "%.9f %.9f %.9f %.9f",
          simplex.area(), simplex.circumference(), simplex.maxWidth(), simplex.minWidth()));
      out.append(' ').append(simplex.equalsCorner(simplex.corner(0)));
      out.append(' ').append(simplex.containsOnBorderLineNo(simplex.corner(0)));
      int[] touching = simplex.touchingSides(other);
      out.append(' ').append(touching.length);
      for (int value : touching) out.append(' ').append(value);
      out.append(' ').append(String.format(Locale.ROOT, "%.9f",
          simplex.distanceToTheLeft(probe)));
      out.append(' ').append(side(simplex.sideOf(probe)));
      out.append(' ').append(simplex.isIntersectedInteriorBy(
          segmentStart, segmentEnd, segmentLine) ? 1 : 0);
      TileShape[] sections = simplex.divideIntoSections(sectionWidth);
      out.append(' ').append(sections.length);
      for (TileShape section : sections) {
        out.append(' ');
        lines(out, section.toSimplex());
      }
      System.out.println(out);
    }
  }
}
