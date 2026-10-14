# Code style of the exercises and solutions

The trainees know Python, but they are not Python experts. They read the solutions to compare them
with their own code, and they read the provided code to understand what happens around their
functions. Every file is written for that reader. `solutions/13_mapping/mapping.py` is the
reference.

## Rules

1. **One step per line.** Build a value, give it a name, then hand the name on. No dict, list or
   object is built inside a function call.

   ```python
   # no
   show.live(lambda: {"motion control": control_state(robot), "body": body_state(robot)})

   # yes
   def read_table() -> dict[str, object]:
       table = {
           "motion control": control_state(robot),
           "body": body_state(robot),
       }
       return table

   show.live(read_table)
   ```

2. **Names say what the value is.** `ids_before`, `is_new`, `fields_by_name`, `rotation`. No single
   letters except the established math names `x`, `y`, `z`, `u`, `v`, `fx`, `fy`, `cx`, `cy` and
   loop counters over plain ranges. No abbreviations like `geo`, `c`, `s`, `px`.

3. **A plain `for` loop before a comprehension.** A short comprehension over one list with one
   condition is fine, e.g. `[entry.id for entry in before]`. Anything with `zip`, a nested call, a
   dict merge (`**`) or more than one line becomes a loop.

4. **No clever idioms.** Avoid:
   - `lambda x=x: ...` to capture a loop variable. Write a small named function or a class method.
   - `f() or "text"` to return a value after a call that returns None. Write two lines.
   - `next(generator)` as a lookup. Write a loop that returns, and raise when nothing was found.
   - `a, b = 1, 2` for unrelated values. One assignment per line.
   - Star unpacking of generators, slices with steps (`words[::2]`), `math.inf` as a flag,
     slicing a tuple of names (`ENDED[1:]`) instead of naming the subset.
   - A dict as a mutable box for closures (`state = {"on": False}`). Use a small class with
     attributes, like `MappingSession` in exercise 13.
   - Lambdas assigned to names. Use `def`.

5. **Named constants instead of magic numbers.** Codes of the GDK (task states, control modes,
   cell values) get a constant with a comment: `TASK_RUNNING = 2`, `JOINT_POSITION_MODE = 1`,
   `CELL_OCCUPIED = 100`.

6. **Types everywhere.** Use the GDK types (`agibot_gdk.MapInfo`, `agibot_gdk.PointCloud`,
   `agibot_gdk.PNCTaskState`) instead of `object`. Use `dict[str, object]` instead of a bare
   `dict`.

7. **Comments explain why.** A comment says what the GDK does that is not obvious, or why the code
   takes a step. It does not repeat the line.

8. **numpy stays vectorised.** Masks and matrix products are part of what the trainees learn, and
   loops over 100 000 lidar points are too slow for the live viewer. Break the expression into named
   steps instead:

   ```python
   rotation = transform[:3, :3]
   translation = transform[:3, 3]
   points_in_base = points @ rotation.T + translation
   ```

## Exercises and solutions

The provided code (everything without a `TODO`) is identical in `exercises/` and `solutions/`. A
change to it is made in both files. The tests in `tests/test_solutions.py` run every solution against
the fake GDK and check that every exercise still raises `NotImplementedError`.
