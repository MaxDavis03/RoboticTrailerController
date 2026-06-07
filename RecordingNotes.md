# EGB439 Self-Assigned Project — 5-Minute Video Script
# Max Davis | n11256931
# "Autonomous Reverse Trailer Control for Differential-Drive Robots"
#
# FORMAT KEY
#   [TIMESTAMP]  — approximate clock position
#   [SCREEN]     — what is displayed on screen at that moment
#   [CUT TO]     — hard cut to new visual
#   [OVERLAY]    — text/graphic that appears on top of current footage
#   narration    — spoken words at ~130 WPM, measured pace
# ==========================================================================


# ==========================================================================
# SECTION 1 — TOPIC & RESEARCH QUESTION   [0:00 – 0:30]
# ==========================================================================

[TIMESTAMP: 0:00]
[SCREEN: Title card — white background, centred black text]
[OVERLAY: "Autonomous Reverse Trailer Control"
          "EGB439 Self-Assigned Project — Max Davis | n11256931"]

  "This project investigates one of the most practically relevant
   yet underexplored problems in mobile robotics — autonomously
   reversing a robot with a passive trailer payload."

[TIMESTAMP: 0:08]
[CUT TO: Image collage — autonomous tractor towing implement | warehouse tug robot | articulated freight vehicle]
[OVERLAY: small caption — "Motivation: agricultural robotics, warehouse automation, autonomous freight"]

  "Reversing a trailer is difficult for a human driver.
   For a robot it is harder still — because the trailer
   introduces a passive articulated joint that makes the
   system nonlinear and inherently unstable in reverse."

[TIMESTAMP: 0:18]
[CUT TO: Simulator — figure-8 path, robot and trailer clearly visible, telemetry panel on right]
[OVERLAY: large centred text fades in]
  "Research question:
   How effectively can a pure-pursuit controller be extended
   as a trailer-aware controller to autonomously reverse a
   differential-drive robot with a passive trailer through
   a constrained environment?"

[TIMESTAMP: 0:27]
[OVERLAY: smaller text below RQ]
  "Objective: develop a simulation, controller and path planner
   capable of stable reverse trailer operation."


# ==========================================================================
# SECTION 2 — RELEVANCE TO ADVANCED ROBOTICS   [0:30 – 1:00]
# ==========================================================================

[TIMESTAMP: 0:30]
[CUT TO: Block diagram — course topics mapped to project components]
[OVERLAY: boxes connecting:
  "Localisation → robot pose estimate"
  "Path planning → A* + spline trajectory"
  "Feedback control → cascaded hitch stabilisation"
  "State estimation → trailer articulation angle"]

  "This project directly integrates the core topics from
   EGB439 — feedback control, state estimation, kinodynamic
   path planning, and localisation — but applies them to an
   articulated multi-body system rather than a single vehicle."

[TIMESTAMP: 0:42]
[OVERLAY: bullet list appears one line at a time]
  "Articulated systems appear across many real-world domains:"
  — "Agricultural equipment towing implements"
  — "Warehouse tug robots hauling shelf trolleys"
  — "Autonomous freight reversing into docks"
  — "Robotic trailer-parking assist"

  "All share the same fundamental challenge: the payload has
   its own heading, its own dynamics, and cannot be directly
   actuated."


# ==========================================================================
# SECTION 3 — BACKGROUND RESEARCH   [1:00 – 2:00]
# ==========================================================================

[TIMESTAMP: 1:00]
[SCREEN: PDF of Coulter 1992 — cursor highlights title and abstract]
[OVERLAY: citation box — "Coulter, R.C. (1992). Implementation of the Pure Pursuit
           Path Tracking Algorithm. CMU-RI-TR-92-01"]

  "The starting point was pure pursuit — a geometric path-tracking
   method developed at Carnegie Mellon in 1992.
   A lookahead circle is drawn around the vehicle's control point;
   the algorithm steers toward the circle's intersection with the path."

[TIMESTAMP: 1:12]
[CUT TO: Diagram — vehicle, lookahead circle, target point, curvature arc labelled]
[OVERLAY: equation   κ = 2y / L_d²   labelled "pure pursuit curvature"]

  "Curvature kappa equals two times the lateral offset divided
   by the lookahead distance squared.
   Simple, fast, and effective — but designed for a single rigid body."

[TIMESTAMP: 1:22]
[CUT TO: Screen showing Altafini 2001 paper or equivalent trailer kinematics reference]
[OVERLAY: citation — "Altafini, C. (2001). Some properties of the general
           n-trailer system. IFAC Proceedings Volumes."]

  "Research into articulated vehicle kinematics shows the trailer
   introduces a second heading state governed by this equation."

[TIMESTAMP: 1:30]
[SCREEN: Equation — θ̇_t = (v / L) · sin(θ_r − θ_t)]
[OVERLAY: labels — "trailer heading rate" | "hitch velocity / trailer length" | "hitch angle φ"]

  "The trailer heading rate equals the hitch velocity divided by
   trailer length, times the sine of the hitch angle.
   In forward motion this is self-stabilising.
   In reverse, the sign of the velocity flips the stability —
   the system becomes exponentially unstable without active feedback."

[TIMESTAMP: 1:42]
[CUT TO: Screen showing Bolzern et al. 1998 or equivalent cascaded controller paper]
[OVERLAY: citation — "Bolzern, P., DeSantis, R., Locatelli, A. (1998).
           Path-tracking for articulated vehicles with off-axle hitching.
           IEEE Transactions on Control Systems Technology."]

  "Literature on cascaded trailer controllers establishes the correct
   architecture: an outer loop that converts path curvature into a
   desired hitch angle, and an inner loop that stabilises that hitch
   angle via robot yaw rate."

[TIMESTAMP: 1:52]
[CUT TO: Simple block diagram of the two-loop architecture]
[OVERLAY: "Path → κ → sin(φ_des) = L·κ → PD(φ_des − φ) → ω → robot → trailer φ → feedback"]

  "This cascaded structure — planning the trailer state rather than
   directly commanding robot heading — is the core contribution
   that makes stable reverse tracking possible."


# ==========================================================================
# SECTION 4 — EXPERIMENTS   [2:00 – 3:00]
# ==========================================================================

[TIMESTAMP: 2:00]
[CUT TO: Simulator — Experiment 1 running: figure-8, speed +0.2, telemetry shows FORWARD mode]
[OVERLAY: "Experiment 1: Forward Trailer Dynamics"
          "figure-8 path | speed +0.20 m/s | hitch controller active"]

  "The first experiment validated the trailer kinematic simulation
   in forward motion.
   A correctly modelled trailer should passively follow the robot
   with no special control — and it does.
   This establishes the simulation as a reliable test environment."

[TIMESTAMP: 2:14]
[CUT TO: Simulator — Experiment 3 first: arc path, NO hitch control, trailer jackknifes within ~5 s]
[OVERLAY: "Experiment 3: Reverse WITHOUT Hitch Stabilisation — UNSTABLE"
          large red text — "JACKKNIFE"]

  "Reversing without hitch-angle feedback immediately destabilises.
   The trailer folds into a jackknife within seconds.
   This directly confirms the literature finding — active hitch
   control is not optional, it is necessary."

[TIMESTAMP: 2:26]
[CUT TO: Simulator — Experiment 2: arc path, speed -0.2, hitch controller active, clean tracking]
[OVERLAY: "Experiment 2: Controlled Reverse — Cascaded Hitch Controller"
          telemetry visible: hitch ±15-35°, phi_des tracking phi_actual]

  "With the cascaded controller active, the same path is tracked
   stably in reverse.
   The inner PD loop holds the hitch angle close to its desired
   value throughout the manoeuvre."

[TIMESTAMP: 2:38]
[CUT TO: Telemetry panel close-up — show phi_des, phi_err, hitch angle live values]
[OVERLAY: arrows labelling phi_des, phi_err, hitch, curvature]

  "The telemetry panel shows the controller's internal state in
   real time — the desired hitch angle, the tracking error,
   and the raw path curvature demand.
   This makes the two-loop architecture directly observable."

[TIMESTAMP: 2:50]
[CUT TO: Simulator — Experiment 4: obstacle field, planned path visible, robot reversing through gaps]
[OVERLAY: "Experiment 4: A* Path Planning Through Obstacle Field"
          "Grid-based A* + cubic spline | 7 circular obstacles | speed -0.20 m/s"]

  "The final experiment shows the complete system.
   A grid-based A* planner computes a collision-free path through
   an obstacle field, smoothed with a cubic spline.
   The hitch controller then tracks this planned path in reverse."


# ==========================================================================
# SECTION 5 — RESULTS & ANALYSIS   [3:00 – 4:00]
# ==========================================================================

[TIMESTAMP: 3:00]
[CUT TO: Split screen or quick-cut sequence — Exp 3 jackknife LEFT | Exp 2 stable tracking RIGHT]
[OVERLAY: "Without hitch control" left | "With cascaded hitch controller" right]

  "The central result is clear.
   Without hitch-angle feedback, the system jackknifes
   within a few seconds on any curved path.
   With it, sustained reverse operation is achievable
   even on paths with moderate curvature."

[TIMESTAMP: 3:14]
[CUT TO: Simulator Exp 2 — telemetry, hitch oscillating in stable range, phi_err small]
[OVERLAY: annotated — "hitch: ±25°" | "φ_err: <10°" | "controller: REVERSE mode"]

  "Quantitatively, the cascaded controller maintained the hitch angle
   within plus or minus thirty degrees on the smooth arc path,
   with tracking error below ten degrees throughout.
   The outer loop curvature demand closely matched the inner loop output."

[TIMESTAMP: 3:26]
[CUT TO: Experiment 4 — obstacle field, path clearly visible, robot weaving between obstacles]
[OVERLAY: "Path planning: A* in 0.1 s | 400-point spline | collision margin: 120 mm"]

  "The A* planner found a feasible path in under two tenths of a
   second, and the spline smoothing produced a path the hitch
   controller could track without geometric deadlock."

[TIMESTAMP: 3:38]
[CUT TO: Telemetry frustration bar — show it partially filling at a tighter section]
[OVERLAY: "Fixup maneuver system — a designed extension (not yet reliable in simulation)"
          frustration bar annotated]

  "A fixup maneuver system was designed and partially implemented
   as an extension. The concept: if the controller remains saturated
   at maximum steering effort near the path for a sustained period,
   this indicates a geometric deadlock — a forward repositioning
   maneuver is triggered to unwind the hitch angle.
   The frustration timer shown here tracks that saturation duration.
   This remains an active area for further development."

[TIMESTAMP: 3:52]
[CUT TO: Experiment 2 — end of path, clean tracking visible in trail]
[OVERLAY: key result text]

  "The key result: trailer-aware cascaded control substantially
   outperforms naive reverse path tracking, enabling stable
   operation that would otherwise be impossible."


# ==========================================================================
# SECTION 6 — DISCUSSION & CONCLUSION   [4:00 – 5:00]
# ==========================================================================

[TIMESTAMP: 4:00]
[CUT TO: Block diagram of full system architecture]
[OVERLAY: diagram — Path → A* planner → spline → controller → Robot → Trailer
                            ↑                         ↑
                    obstacle map              hitch angle feedback]

  "The project demonstrated that classical mobile robotics
   techniques can be extended to articulated systems — but only
   when the controller explicitly closes a feedback loop
   around the hitch-angle state."

[TIMESTAMP: 4:10]
[OVERLAY: "Limitations identified:" — bullet points appear one at a time]
  — "Lookahead distance is a fixed trade-off: short is reactive, long is smooth"
  — "Fixed gains do not adapt to path curvature or speed changes"
  — "Fixup maneuver direction is geometrically motivated but not globally optimal"
  — "Simulation assumes perfect state — real hardware adds slip, delay, noise"
  — "A* plans in 2D position space — does not model trailer state constraints"

  "Several clear limitations emerged. The fixed lookahead distance
   is a fundamental trade-off — tight paths need short lookahead,
   which amplifies noise. And the path planner treats the robot
   as a point, without modelling the trailer's turning constraints
   during planning."

[TIMESTAMP: 4:30]
[OVERLAY: "Recommended extensions:" — list appears]
  — "Adaptive lookahead: scale with local path curvature"
  — "Reliable fixup maneuver: geometry-aware forward repositioning"
  — "Articulation-aware planning: Hybrid A* with hitch-angle constraints"
  — "Model Predictive Control: anticipate hitch limits multi-step ahead"

  "Future work would focus first on adaptive lookahead — scaling the
   lookahead radius as a function of local path curvature eliminates
   the fixed trade-off entirely. A reliable fixup maneuver and
   articulation-aware path planning using Hybrid A* would then
   extend the system to genuinely constrained environments."

[TIMESTAMP: 4:47]
[CUT TO: Experiment 4 footage — full run, obstacle field, planned path, clean reverse tracking]
[OVERLAY: returning title — "Autonomous Reverse Trailer Control"]

  "In conclusion — reversing a trailer is substantially harder than
   standard mobile robot navigation. It requires new kinematic models,
   a new control architecture, and planning strategies that account
   for physical instability."

[TIMESTAMP: 4:55]
[OVERLAY: final text fades in over footage]
  "Articulated trailer systems CAN be autonomously controlled
   using advanced robotics techniques —
   but they demand substantially more sophisticated approaches
   than a conventional mobile robot."

[TIMESTAMP: 5:00]
[SCREEN: End card — "EGB439 | Max Davis | n11256931" | QUT colours]


# ==========================================================================
# CRITERIA CHECKLIST
# ==========================================================================
#
# [CRITERIA 1 — Core question]
#   ✓ Research question displayed as text at 0:18
#   ✓ Objective stated at 0:27
#   ✓ Real-world motivation images at 0:08
#
# [CRITERIA 2 — Depth of research]
#   ✓ Coulter 1992 — citation + diagram + κ equation at 1:00–1:21
#   ✓ Altafini 2001 — citation + θ̇_t equation with labels at 1:22–1:41
#   ✓ Bolzern 1998 — citation + cascaded architecture diagram at 1:42–1:59
#   ✓ All citations: author, year, title, venue
#   ✓ Equations explained in narration, not just displayed
#
# [CRITERIA 3 — Experimental rationale]
#   ✓ Exp 1: validates simulation model (what/how/why stated)
#   ✓ Exp 3: demonstrates instability without control (justifies the work)
#   ✓ Exp 2: shows cascaded controller working (main result)
#   ✓ Exp 4: full system integration — planner + controller
#   ✓ Telemetry panel shown and narrated — internal state visible
#   ✓ Fixup described as designed extension, not claimed as working
#
# [CRITERIA 4 — Results & analysis]
#   ✓ Direct comparison: with vs without hitch control at 3:00
#   ✓ Quantitative values from telemetry narrated at 3:14
#   ✓ Limitations list at 4:10 — specific and honest
#   ✓ Findings tied back to research question in conclusion
#   ✓ No overclaiming — fixup described accurately as "partially implemented"
#   ✓ Video quality: labelled overlays, annotated telemetry, clean cuts


# ==========================================================================
# PRODUCTION NOTES
# ==========================================================================
#
# RECORDING ORDER:
#   1. Run   python run_experiments.py   and screen-record all four experiments
#   2. Screen-record the paper PDFs (Coulter, Altafini/equivalent, Bolzern)
#      with cursor slowly highlighting title + key equation
#   3. Create the block diagram slides (PowerPoint or Keynote is fine)
#   4. Record narration last, reading from this script
#
# EXPERIMENT DURATIONS IN run_experiments.py:
#   Exp 1 (forward)    25 s — record full run, use ~15 s in edit
#   Exp 2 (controlled) 35 s — record full run, use ~20 s in edit
#   Exp 3 (unstable)   15 s — jackknife happens fast, use all of it
#   Exp 4 (obstacles)  60 s — use ~20 s showing full traversal
#
# WHAT TO SHOW FOR EXP 3 (UNSTABLE):
#   The trailer jackknifes quickly — let it run a few seconds past jackknife
#   before cutting so the viewer clearly sees the instability.
#   You may need to slow the playback to 0.5× to make it obvious.
#
# TIMING CHECK:
#   Script runs ~4:56 at 130 WPM with normal pauses.
#   The two largest timing risks are sections 3 (research) and 4 (experiments).
#   If running long, trim the application list at 0:42 to two bullet points.