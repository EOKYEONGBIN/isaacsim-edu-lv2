# Changelog

## [0.1.0] - 2026-11-12
- Lv2 Lesson 1, copied from `Isaacsim_Edu_Lv1_Lesson3` 0.53.0 (with its uncommitted working-tree changes as of
  2026-11-12: camera home reset etc. used by the forklift AI). Renamed the extension, module, class, window title
  (`AZ Edu Lv2 - Lesson 1`), dock target (`AZ Edu Lv2`), active-lesson setting and log prefixes.
- Kept the Action Graph node type names (`Isaacsim_Edu_Lv1_Lesson3.Forklift*`): the scene files refer to them.
  On startup every enabled Lv1 lesson is turned off, so the two copies never register them at the same time.
- Earlier history: see `Isaacsim_Edu_Lv1_Lesson3/docs/CHANGELOG.md` in the isaacsim-edu-lv1 repository.
