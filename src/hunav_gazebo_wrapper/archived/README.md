# Archived Files

This directory contains outdated configurations, scenarios, and behavior tree files that were superseded by the **One-At-A-Time (OAT) Sensitivity Analysis** methodology implemented in March 2026.

## Contents

### `archived/scenarios/`

**Archived Scenario Files:**
- `agents_cafe.yaml` - Original static café configuration (3 agents)
- `agents_house.yaml` - Original static house configuration (3 agents)
- `agents_warehouse.yaml` - Original static warehouse configuration (3 agents)

**Why Archived:**
- Replaced by OAT approach with 7 parametric variations per world
- Old files used fixed parameters rather than systematic sensitivity analysis
- Crowd density was not scientifically justified (3 agents per world)

### `archived/behavior_trees/`

**Archived Behavior Tree Files:**
- `agents_house__agent_1_bt.xml`, `_2_bt.xml`, `_3_bt.xml` - House behavior trees (3 agents)
- `agents_warehouse__agent_1_bt.xml`, `_2_bt.xml`, `_3_bt.xml` - Warehouse behavior trees (3 agents)

**Why Archived:**
- Replaced by organized world-specific structure: `behavior_trees/[world]/`
- Old files used fixed agent counts (3) rather than scalable templates
- No longer needed; current system uses `agents_[world]__agent_[id]_bt.xml` pattern with dynamic templating

## Current Structure (Active)

### Active Scenarios
```
scenarios/domenic/cafe/
├── cafe_oat_balanced_baseline.yaml
├── cafe_oat_social_low.yaml
├── cafe_oat_social_high.yaml
├── cafe_oat_goal_low.yaml
├── cafe_oat_goal_high.yaml
├── cafe_oat_speed_slow.yaml
└── cafe_oat_speed_fast.yaml
```

### Active Behavior Trees
```
behavior_trees/cafe/
├── agents_cafe__agent_1_bt.xml
├── agents_cafe__agent_2_bt.xml
├── ...
└── agents_cafe__agent_12_bt.xml
```

## When These Files Were Replaced

**Date:** March 29, 2026
**Reason:** Transition to systematic OAT sensitivity analysis
**Improvement:** 
- Parametric variations: 7 configurations (1 baseline + 6 variations)
- Agent count: 3 → 12 (realistic crowd density)
- Testing scope: 3 configs × 1 run → 7 configs × 3 runs = 21 runs
- Methodology: Fixed → Data-driven parameter selection

## Recovering Archived Files

If you need to restore an archived file (not recommended):

```bash
# Restore a specific scenario
cp archived/scenarios/agents_cafe.yaml scenarios/domenic/cafe/agents_cafe_original.yaml

# Restore behavior trees
cp archived/behavior_trees/agents_warehouse__agent_*.xml behavior_trees/warehouse/
```

## Cleanup Notes

**Deleted (not archived):**
- Agents 13-25 behavior tree files for café (`agents_cafe__agent_13_bt.xml` through `_25_bt.xml`)
  - Reason: Reduced from 25 to 12 agents for realistic café density
  - Timestamp: March 29, 2026, 13:00 UTC

**Files to Keep:**
- All OAT scenario files in `scenarios/domenic/cafe/`
- All 12 behavior trees in `behavior_trees/cafe/`
- World-specific placeholder directories ready for future expansion

---

**Last Updated:** March 29, 2026
**Status:** Legacy files - do not use for active testing
