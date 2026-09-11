/*
 * GPL-3.0-or-later.  Parse-only helper for the Freerouting parity manifest.
 *
 * This deliberately uses the JDK compiler tree API instead of a regular
 * expression.  The pinned source contains nested classes, records, generic
 * methods, annotations and multiline declarations; silently missing one of
 * those would defeat the purpose of a method-complete inventory.
 */

import com.sun.source.tree.ClassTree;
import com.sun.source.tree.CompilationUnitTree;
import com.sun.source.tree.MethodTree;
import com.sun.source.tree.Tree;
import com.sun.source.util.JavacTask;
import com.sun.source.util.SourcePositions;
import com.sun.source.util.TreePathScanner;
import com.sun.source.util.Trees;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Path;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Deque;
import java.util.List;
import java.util.Locale;
import javax.tools.JavaCompiler;
import javax.tools.JavaFileObject;
import javax.tools.StandardJavaFileManager;
import javax.tools.ToolProvider;

/** Emits one JSON record per declared type and method. */
public final class JavaMethodManifest {
  private JavaMethodManifest() {}

  private static String json(String value) {
    StringBuilder result = new StringBuilder(value.length() + 16);
    result.append('"');
    for (int index = 0; index < value.length(); ++index) {
      char current = value.charAt(index);
      switch (current) {
        case '"' -> result.append("\\\"");
        case '\\' -> result.append("\\\\");
        case '\b' -> result.append("\\b");
        case '\f' -> result.append("\\f");
        case '\n' -> result.append("\\n");
        case '\r' -> result.append("\\r");
        case '\t' -> result.append("\\t");
        default -> {
          if (current < 0x20) {
            result.append(String.format(Locale.ROOT, "\\u%04x", (int) current));
          } else {
            result.append(current);
          }
        }
      }
    }
    return result.append('"').toString();
  }

  private static String normalized(Tree tree) {
    return tree.toString().replaceAll("\\s+", " ").trim();
  }

  private static final class Scanner extends TreePathScanner<Void, Void> {
    private final CompilationUnitTree unit;
    private final SourcePositions positions;
    private final Path sourceRoot;
    private final Deque<String> owners = new ArrayDeque<>();
    private int anonymousIndex;

    Scanner(
        CompilationUnitTree unit,
        SourcePositions positions,
        Path sourceRoot) {
      this.unit = unit;
      this.positions = positions;
      this.sourceRoot = sourceRoot;
    }

    private long line(Tree tree) {
      long position = positions.getStartPosition(unit, tree);
      return position < 0 ? -1 : unit.getLineMap().getLineNumber(position);
    }

    private String owner() {
      String packageName = unit.getPackageName() == null ? "" : unit.getPackageName().toString();
      String nested = String.join("$", owners);
      return packageName.isEmpty() ? nested : packageName + "." + nested;
    }

    private String path() {
      Path source = Path.of(unit.getSourceFile().toUri()).toAbsolutePath().normalize();
      return sourceRoot.relativize(source).toString().replace('\\', '/');
    }

    @Override
    public Void visitClass(ClassTree tree, Void ignored) {
      String simpleName = tree.getSimpleName().toString();
      if (simpleName.isEmpty()) {
        simpleName = "<anonymous@" + line(tree) + ":" + (++anonymousIndex) + ">";
      }
      owners.addLast(simpleName);
      System.out.println(
          "{\"record\":\"class\",\"path\":"
              + json(path())
              + ",\"owner\":"
              + json(owner())
              + ",\"kind\":"
              + json(tree.getKind().name())
              + ",\"line\":"
              + line(tree)
              + "}");
      super.visitClass(tree, ignored);
      owners.removeLast();
      return null;
    }

    @Override
    public Void visitMethod(MethodTree tree, Void ignored) {
      List<String> parameterTypes = new ArrayList<>();
      tree.getParameters().forEach(parameter -> parameterTypes.add(normalized(parameter.getType())));
      String name = tree.getName().toString();
      boolean constructor = name.equals("<init>");
      String signature =
          (constructor ? owners.peekLast() : name)
              + "("
              + String.join(",", parameterTypes)
              + ")";
      System.out.println(
          "{\"record\":\"method\",\"path\":"
              + json(path())
              + ",\"owner\":"
              + json(owner())
              + ",\"name\":"
              + json(name)
              + ",\"signature\":"
              + json(signature)
              + ",\"line\":"
              + line(tree)
              + ",\"constructor\":"
              + constructor
              + "}");
      return super.visitMethod(tree, ignored);
    }
  }

  public static void main(String[] arguments) throws IOException {
    if (arguments.length < 2 || !arguments[0].equals("--source-root")) {
      System.err.println("usage: JavaMethodManifest --source-root ROOT SOURCE.java...");
      System.exit(2);
    }

    Path sourceRoot = Path.of(arguments[1]).toAbsolutePath().normalize();
    List<String> sources = Arrays.asList(arguments).subList(2, arguments.length);
    JavaCompiler compiler = ToolProvider.getSystemJavaCompiler();
    if (compiler == null) {
      throw new IllegalStateException("a full JDK is required to parse the Freerouting source");
    }

    try (StandardJavaFileManager files =
        compiler.getStandardFileManager(null, Locale.ROOT, StandardCharsets.UTF_8)) {
      Iterable<? extends JavaFileObject> units = files.getJavaFileObjectsFromStrings(sources);
      JavacTask task =
          (JavacTask)
              compiler.getTask(
                  null,
                  files,
                  diagnostic -> {
                    // Parsing does not resolve symbols.  Ignore attribution diagnostics.
                  },
                  List.of("-proc:none"),
                  null,
                  units);
      Trees trees = Trees.instance(task);
      SourcePositions positions = trees.getSourcePositions();
      for (CompilationUnitTree unit : task.parse()) {
        new Scanner(unit, positions, sourceRoot).scan(unit, null);
      }
    }
  }
}
