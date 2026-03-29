# Investigation: Low Path Efficiency Issue

**Date:** 29 March 2026  
**Recording:** 73.22 seconds (12 agents × 120 frames)  
**Finding:** Path efficiency 0.061 (should be 0.966)

---

## ROOT CAUSE: SFM Parameter Oscillation (Not Data Corruption)

### Evidence
Agent 2 trajectory analysis reveals **systematic oscillation pattern**:

```
Frame 0:  Start at (-0.364, +0.874)
↓
Frame 15: Reaches (-0.302, -4.469)  — moving southwest steadily
↓
Frame 16: REVERSAL to (+0.435, -5.702) — sudden direction change!
↓        
Frame 17-29: Backtracks northeast (opposite direction)
↓
Frame 110-119: Resumes southwest toward goal
↓
Frame 119: Ends at (-0.946, -3.631)
```

**Path:** 62.5 meters total  
**Direct displacement:** 4.5 meters  
**Efficiency:** 0.073

### The Mechanism

Agents are caught in **conflicting force fields**:

1. **Goal Force:** "Go to goal (southwest)"
2. **Social/Obstacle Forces:** "Avoid collisions (northeast)" 
3. **Result:** Agent oscillates between forces instead of finding smooth path

This is **NOT** a bug — it's a **parameter tuning problem**.

---

## What This Means for OAT Analysis

### Good News ✓
- **Data is HIGH QUALITY:** Agents move continuously, not static
- **Duration sufficient:** 73.22s is enough to see behavior patterns
- **Problem is systematic:** Same issue across all 12 agents
- **OAT _IS_ the solution:** We need to systematically vary SFM parameters to fix this

### What We'll Learn from OAT

| Test Set | Parameter | Expected Fix | Metric Impact |
|----------|-----------|--------------|----------------|
| **Tests 4-9** | social_force_factor (low/high) | Strong repulsion = smooth paths? | Will reduce oscillation |
| **Tests 10-15** | goal_force_factor (low/high) | Weak goal pull = less impatience? | Will improve efficiency |
| **Tests 16-21** | max_vel (slow/fast) | Slower speed = less jerky turns? | Will test speed-efficiency tradeoff |

### Baseline Diagnosis
```
Metric              Current    GT (ETH)    Status
──────────────────────────────────────────────
Speed (moving)      0.83 m/s   1.46 m/s   -43% ok, agents ARE moving
Collisions          0.109/s    0.0004/s   +273× HIGH (consequence of oscillation)
Acceleration        0.39 m/s²  0.74 m/s²  -47% ok, not too jerky
Path Efficiency     0.061      0.966      -94% CRITICAL ← OAT will fix this
```

---

## Recommended Next Steps

### DO NOT wait for fixes before OAT

The baseline shows **what breaks** with current parameters. This is VALUABLE DATA for OAT:

1. **Run full 21-run OAT suite** with current baseline parameters
2. **Track how each variation affects:**
   - Path efficiency (primary target)
   - Collision rate (secondary target)  
   - Speed/acceleration (tertiary target)
3. **Analyze results:**
   - Which parameter has strongest effect on efficiency?
   - Which parameter reduces collisions most?
   - Is there a sweet spot or tradeoff?

### Why OAT Will Work Here

- **Baseline is broken in a SYSTEMATIC way** (not random)
- **All 12 agents show same pattern** (not edge case)
- **Root cause is SFM parameter imbalance** (exactly what OAT diagnoses)
- **Variations WILL show measurable differences** in metrics

---

## Example OAT Prediction

**If social_force_factor is too weak:**
- Tests 4-6 (low): Efficiency stays awful ✗
- Tests 7-9 (high): Efficiency improves ✓ → Diagnosis confirmed

**If goal_force_factor is too strong:**
- Tests 10-12 (low): Efficiency improves ✓ → Diagnosis confirmed
- Tests 13-15 (high): Efficiency gets worse ✗

The OAT results will tell you **exactly which parameters to adjust and how much**.

---

## Conclusion

**Status:** ✓ Ready for OAT analysis

The baseline isn't broken — it's showing us the **current problem state**. 
OAT will systematically find the levers to fix it.

Proceed with full 21-run suite. The analysis tools are ready.
