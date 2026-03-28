# Experimental Workflow: SFM Sensitivity Analysis

Systematic parameter testing for the café_no_robot baseline to identify which Social Force Model (SFM) parameters drive realistic pedestrian metrics.

## Approach: One-At-A-Time (OAT) Sensitivity Analysis

**Goal:** Vary individual parameters while holding others constant to isolate cause-and-effect relationships.

**Test Plan:**
- **Tests 1-3:** Balanced baseline (benchmark)
- **Tests 4-9:** Vary social force (person-to-person collision sensitivity)
- **Tests 10-15:** Vary goal force (path efficiency/directedness)
- **Tests 16-21:** Vary walking speed (speed-collision tradeoff)

**Total:** 21 runs (~2-2.5 hours)

## Quick Start

### 1. Build
```bash
cd ~/sfm_ws_fresh
colcon build --packages-select hunav_gazebo_wrapper hunav_evaluator
source install/setup.bash
```

### 2. Run Tests
Start with balanced baseline tests (1-3), then proceed through variations.

**Quick reference:** [OAT_QUICK_REF.md](OAT_QUICK_REF.md) — Copy-paste commands for all 21 tests

**Full methodology:** [OAT_SENSITIVITY_GUIDE.md](OAT_SENSITIVITY_GUIDE.md) — Complete guide with setup & interpretation

## Key Files

- `generate_oat_scenarios.py` — Generates all 7 test scenario files
- `src/hunav_gazebo_wrapper/scenarios/domenic/cafe_oat_*.yaml` — Test configurations (7 files)
- `compare_results.py` — Analyzes results after runs complete

## After All Tests

Compare your results to ETH/UCY ground truth:
```bash
python3 src/DATASETS/domenic_analysis/compare_results.py
```

## Workflow

1. Run Tests 1-3 (Balanced Baseline)
2. Reflect: Which metrics need improvement?
3. Run Tests 4-21 (Variations) to isolate sensitive parameters
4. Analyze: Which parameter changes helped most?
5. Design final profile combining best findings
6. Document methodology for thesis report

---

For detailed instructions, see [OAT_SENSITIVITY_GUIDE.md](OAT_SENSITIVITY_GUIDE.md).
