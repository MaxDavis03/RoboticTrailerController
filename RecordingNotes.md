# EGB439 Self-Assigned Project — 5-Minute Video Script
# Max Davis | n11256931
# "Autonomous Reverse Trailer Control for Differential-Drive Robots"
#
# FORMAT KEY
#   [TIMESTAMP]  — approximate clock position
#   [SCREEN]     — what is displayed on screen
#   [CUT TO]     — hard cut to new visual
#   [OVERLAY]    — text/graphic that appears on top of current footage
#   (narration)  — spoken words, spoken at measured pace ~2.2 words/sec
# ==========================================================================


# ==========================================================================
# SECTION 1 — TOPIC & RESEARCH QUESTION   [0:00 – 0:30]
# ==========================================================================

[TIMESTAMP: 0:00]
[SCREEN: Clean title card — white background, black text]
[OVERLAY: "Autonomous Reverse Trailer Control" | "EGB439 Self-Assigned Project" | "Max Davis — n11256931"]

  "This project investigates one of the most practically relevant
   yet underexplored problems in mobile robotics — autonomously
   reversing a robot with a passive trailer payload."

[TIMESTAMP: 0:07]
[CUT TO: Real-world footage or image — truck reversing a trailer, or forklift with tow attachment]
[OVERLAY: small text bottom-left — "Motivation: autonomous logistics, agricultural robotics, warehouse tug systems"]

  "Reversing a trailer is something most people find difficult.
   For a robot, it's even harder — because the trailer introduces
   a passive articulated joint that makes the system nonlinear
   and inherently unstable in reverse."

[TIMESTAMP: 0:18]
[CUT TO: Simulator running — figure-8 or hairpin path, robot and trailer clearly visible]
[OVERLAY: Large centred text — research question, fades in over 1 second]

  "The research question is:
   How effectively can a pure-pursuit vehicle controller be extended
   as a trailer-aware controller, to autonomously reverse a
   differential-drive robot with a passive trailer through a
   constrained environment?"

[TIMESTAMP: 0:28]
[OVERLAY: smaller text under RQ — "Objective: develop a controller + path planner for stable reverse trailer operation"]

  "The project objective was to build a complete system — simulation,
   controller, and path planner — capable of reversing a trailer
   through obstacle fields while maintaining stability."


# ==========================================================================
# SECTION 2 — RELEVANCE TO ADVANCED ROBOTICS   [0:30 – 1:00]
# ==========================================================================

[TIMESTAMP: 0:30]
[CUT TO: Split-screen collage — autonomous tractor in crop row | warehouse AMR towing shelves | autonomous truck]
[OVERLAY: "Real-world relevance"]

  "Articulated vehicle systems appear across a wide range of
   autonomous robotics domains."

[TIMESTAMP: 0:35]
[OVERLAY: Bullet points appear one at a time as narrated]
  — "Autonomous agricultural equipment towing implements through crop rows"
  — "Warehouse tug robots hauling shelf-trolley payloads"
  — "Last-mile freight vehicles reversing into loading docks"
  — "Robotic trailer-parking assist systems"

  "All of these share the same fundamental challenge: the payload
   has its own heading, its own dynamics, and it cannot be directly
   actuated."

[TIMESTAMP: 0:48]
[CUT TO: Block diagram — showing course topics connected to project components]
[OVERLAY: Diagram showing: "Localisation → robot pose" | "Path planning → trajectory" | "Feedback control → hitch stabilisation" | "State estimation → trailer angle"]

  "This project directly integrates the advanced robotics topics
   covered this semester — feedback control, state estimation,
   kinodynamic path planning, and localisation — but applies
   them to an articulated multi-body system rather than a
   single rigid vehicle."

  "That extension is non-trivial. It requires new kinematic models,
   a new control architecture, and planning strategies that account
   for reversing stability constraints."


# ==========================================================================
# SECTION 3 — BACKGROUND RESEARCH & LIT REVIEW   [1:00 – 2:00]
# ==========================================================================

[TIMESTAMP: 1:00]
[CUT TO: Screen showing a PDF or paper — Coulter 1992 Pure Pursuit]
[OVERLAY: Citation — "Coulter, R.C. (1992). Implementation of the Pure Pursuit Path Tracking Algorithm. CMU-RI-TR-92-01"]

  "The starting point was pure pursuit — a geometric path-tracking
   method developed at CMU in 1992. The algorithm defines a lookahead
   circle around the vehicle's control point, finds where that circle
   intersects the path, and steers toward that intersection."

[TIMESTAMP: 1:12]
[SCREEN: Diagram showing pure pursuit geometry — vehicle, lookahead circle, target point, curvature arc]
[OVERLAY: Equation — κ = 2y / L_d² labelled "pure pursuit curvature"]

  "Curvature kappa equals two times the lateral offset divided by
   the lookahead distance squared. This is simple and fast,
   but it was designed for a single rigid vehicle — not an
   articulated system."

[TIMESTAMP: 1:22]
[CUT TO: Screen showing paper — Altafini (2001) or similar trailer kinematics paper]
[OVERLAY: Citation — "Altafini, C. (2001). Some properties of the general n-trailer system. IFAC Proceedings."]

  "Research into articulated vehicle kinematics shows that the
   trailer introduces a second heading state — the trailer angle —
   governed by this differential equation."

[TIMESTAMP: 1:30]
[SCREEN: Equation displayed — θ̇_t = (v/L) · sin(θ_r - θ_t)]
[OVERLAY: Labels — "trailer heading rate" | "speed at hitch" | "trailer length" | "hitch angle φ"]

  "The trailer heading rate equals the hitch-point velocity divided
   by the trailer length, times the sine of the hitch angle.
   In forward motion this is self-stabilising. In reverse, the
   sign flips and it becomes exponentially unstable without active
   hitch-angle feedback."

[TIMESTAMP: 1:42]
[CUT TO: Screen showing paper — Sauerbeck / Bolzern et al. or similar cascaded trailer controller paper]
[OVERLAY: Citation — "Bolzern, P., DeSantis, R., Locatelli, A. (1998). Path-tracking for articulated vehicles. IEEE T-CST."]

  "Literature on cascaded trailer controllers shows that the
   correct architecture has two loops: an outer loop that converts
   path curvature into a desired hitch angle, and an inner loop
   that stabilises that hitch angle by controlling robot yaw rate."

[TIMESTAMP: 1:52]
[CUT TO: Screen showing Garrido-Jurado et al. ArUco paper or OpenCV ArUco docs]
[OVERLAY: Citation — "Garrido-Jurado et al. (2014). Automatic generation of fiducial markers. Pattern Recognition Letters."]

  "For hitch angle measurement on the physical robot, this project
   used ArUco fiducial markers and a rear-facing camera to estimate
   trailer articulation in real time — an approach validated in the
   robotics literature for low-cost pose estimation."


# ==========================================================================
# SECTION 4 — EXPERIMENTS   [2:00 – 3:00]
# ==========================================================================

[TIMESTAMP: 2:00]
[CUT TO: Simulator — forward motion, clean tracking, trailer following smoothly]
[OVERLAY: "Experiment 1: Forward trailer dynamics" | "Mode: FORWARD" shown in telemetry]

  "The first experiment validated the trailer kinematic model in
   forward motion. A correctly modelled trailer should passively
   follow the robot with no active control — and with the correct
   hitch-velocity formulation, it does."

[TIMESTAMP: 2:10]
[CUT TO: Simulator — reverse motion WITHOUT hitch control, trailer jackknifing immediately]
[OVERLAY: "Experiment 2: Reverse without hitch stabilisation — UNSTABLE"]

  "Reversing without hitch-angle feedback immediately destabilises.
   The trailer folds rapidly into a jackknife. This confirmed
   the literature finding and justified the need for active control."

[TIMESTAMP: 2:20]
[CUT TO: Simulator — reverse with hitch controller active, smooth tracking on a gentle curve]
[OVERLAY: "Experiment 3: Cascaded hitch-angle controller" | telemetry shows hitch angle oscillating stably ~15-30°]

  "Adding the cascaded hitch controller — outer loop curvature
   to desired hitch angle, inner loop PD on hitch error —
   produced stable reversing on gentle curves, with the hitch
   angle remaining well within bounds."

[TIMESTAMP: 2:33]
[CUT TO: Simulator — hairpin or demo path, frustration bar filling, fixup triggering, robot repositioning forward then resuming reverse]
[OVERLAY: "Experiment 4: Fixup maneuver system" | frustration bar highlighted | "FIXUP" mode shown]

  "For tight corners where the required hitch angle exceeds what the
   controller can achieve, a fixup state machine was implemented.
   It triggers only when the controller has been saturated at maximum
   steering effort for a sustained period near the path — confirming
   a genuine geometric deadlock rather than a transient."

[TIMESTAMP: 2:47]
[CUT TO: Real robot footage — if available — robot reversing with trailer, or ArUco hitch estimation camera view]
[OVERLAY: "Experiment 5: Vision-based hitch angle estimation" | ArUco detection overlay visible on camera feed]

  "On the physical platform, hitch angle was estimated using ArUco
   markers on the trailer, detected by a rear-facing camera. This
   provided real-time articulation state without additional mechanical
   sensors."

[TIMESTAMP: 2:55]
[CUT TO: Quick montage — path planner generating a path through an obstacle field (or show code/plot if hardware not available)]
[OVERLAY: "Experiment 6: Path planning through constrained environments"]

  "Finally, path planning was extended to generate reversing
   trajectories through constrained obstacle environments,
   considering minimum turning radius, maximum hitch angle,
   and recovery maneuver feasibility."


# ==========================================================================
# SECTION 5 — RESULTS & ANALYSIS   [3:00 – 4:00]
# ==========================================================================

[TIMESTAMP: 3:00]
[CUT TO: Simulator — forward tracking, telemetry panel visible, hitch angle low and stable]
[OVERLAY: "Result: Forward motion — stable, low tracking error"]

  "Forward trailer motion was naturally stable and easy to control.
   The trailer passively aligned with the robot heading within
   one to two body lengths, consistent with the kinematic model."

[TIMESTAMP: 3:10]
[CUT TO: Simulator — reverse on figure-8 or arc path, hitch angle oscillating in mid-range, tracking correctly]
[OVERLAY: Telemetry showing φ_des tracking φ_actual | "REVERSE" mode | hitch ±20-35°]

  "With the cascaded controller active, reverse tracking was
   significantly more stable. The inner loop kept hitch angle
   close to the desired value, and the frustration trigger
   prevented spurious fixups during normal operation."

[TIMESTAMP: 3:22]
[CUT TO: Demo/hairpin path — fixup triggered, forward maneuver clearly visible, resumes reversing]
[OVERLAY: "Result: Fixup maneuver successfully recovers from geometric deadlock" | frustration bar annotated]

  "The fixup system demonstrated reliable recovery from tight corners.
   The three-condition trigger — large heading error, small distance
   to path, and sustained saturation — prevented false positives
   while ensuring recovery when genuinely needed."

[TIMESTAMP: 3:34]
[CUT TO: Real robot or ArUco video — hitch angle estimate plotted or shown numerically]
[OVERLAY: "Result: Vision-based hitch estimation — accurate, real-time"]

  "ArUco-based hitch angle estimation tracked trailer articulation
   in real time at camera frame rate. Accuracy was sufficient for
   closed-loop hitch feedback, though sensitivity to lighting and
   partial occlusion was noted."

[TIMESTAMP: 3:44]
[CUT TO: Telemetry panel in simulator — show frustration bar, mode switching, phi_des and phi_err]
[OVERLAY: Annotated telemetry — arrows labelling each field]

  "The telemetry system provided real insight into controller
   behaviour — the frustration bar made the saturation timer
   visible, and the phi_des versus phi_err readout directly
   showed the inner-loop tracking quality."

[TIMESTAMP: 3:52]
[CUT TO: Side-by-side or sequence — naive reverse tracking failing vs cascaded controller succeeding on same path]
[OVERLAY: "Naive reverse tracking" left | "Cascaded hitch controller" right]

  "The key finding is that trailer-aware planning and control
   substantially outperforms naive reverse path tracking.
   Without hitch feedback, the system jackknifes within seconds.
   With it, sustained reverse operation is achievable even
   on paths with moderate curvature."


# ==========================================================================
# SECTION 6 — DISCUSSION & CONCLUSION   [4:00 – 5:00]
# ==========================================================================

[TIMESTAMP: 4:00]
[CUT TO: Clean diagram — two-layer control architecture with labels]
[OVERLAY: Block diagram — "Path" → "Curvature κ" → "Desired hitch φ_des" → "PD controller" → "Robot ω" → "Trailer θ_t" feedback loop]

  "The project demonstrated that classical mobile robotics
   techniques can be extended to articulated systems — but only
   when the controller explicitly models the hitch-angle state
   and closes a feedback loop around it."

[TIMESTAMP: 4:12]
[CUT TO: List of limitations appearing one by one]
[OVERLAY: "Limitations identified:"]
  — "Sensitivity to noisy hitch-angle estimates"
  — "Lookahead distance is a fixed trade-off — short is reactive, long is smooth"
  — "Fixup maneuver direction is geometric but not globally optimal"
  — "Wheel slip and odometry drift affect the real robot"
  — "No real-time obstacle avoidance once path is set"

  "Several limitations were identified. Hitch angle estimation
   is noisy at large distances. The fixed lookahead distance is
   a fundamental trade-off — tighter paths need shorter lookahead,
   which amplifies noise. And the fixup maneuver direction, while
   geometrically motivated, is not globally optimal."

[TIMESTAMP: 4:30]
[CUT TO: Future work — show one or two example concepts visually]
[OVERLAY: "Future directions:"]
  — "Model Predictive Control for constrained multi-step planning"
  — "Adaptive lookahead as a function of path curvature"
  — "SLAM-integrated trailer tracking for real-world deployment"
  — "Multi-trailer articulated systems"

  "Future work could address these limitations. Adaptive lookahead
   based on local path curvature would eliminate the fixed trade-off.
   Model predictive control would allow the planner to anticipate
   hitch angle constraints before they become critical. And
   SLAM integration would support real-world deployment where
   localiser infrastructure is unavailable."

[TIMESTAMP: 4:47]
[CUT TO: Final simulator run — clean reverse tracking with telemetry visible, fixup triggering once, recovering cleanly]
[OVERLAY: Title card returning — "Autonomous Reverse Trailer Control"]

  "In conclusion — reversing a trailer is a substantially harder
   robotics problem than standard mobile robot navigation.
   It requires new kinematic models, a new control architecture,
   and planning strategies that account for physical instability."

[TIMESTAMP: 4:55]
[OVERLAY: Final text block fades in over simulator footage]
  "Articulated trailer systems CAN be autonomously controlled
   using advanced robotics techniques —
   but they demand substantially more sophisticated approaches
   than a conventional mobile robot."

[TIMESTAMP: 5:00]
[SCREEN: End card — QUT logo, unit code EGB439, name, student number]


# ==========================================================================
# SCREEN CONTENT CHECKLIST (match to criteria)
# ==========================================================================
#
# [CRITERIA 1 — Core question]
#   ✓ Research question displayed as text overlay at 0:18
#   ✓ Objective stated at 0:28
#   ✓ Real-world motivation footage/images at 0:07
#
# [CRITERIA 2 — Depth of research]
#   ✓ Coulter (1992) pure pursuit — citation + diagram + equation at 1:00–1:21
#   ✓ Altafini (2001) trailer kinematics — citation + equation at 1:22–1:41
#   ✓ Bolzern et al. (1998) cascaded controller — citation + architecture at 1:42–1:51
#   ✓ Garrido-Jurado (2014) ArUco — citation + camera footage at 1:52–1:59
#   ✓ All citations include author, year, title, venue
#
# [CRITERIA 3 — Experimental rationale]
#   ✓ Six distinct experiments with stated purpose and expected outcome
#   ✓ Each experiment links back to a research finding or course concept
#   ✓ "What": the experiment | "How": the setup/method | "Why": the insight
#   ✓ Telemetry panel shown throughout to make internal state visible
#
# [CRITERIA 4 — Results & analysis]
#   ✓ Quantitative telemetry shown (hitch angle values, phi_des vs phi_err)
#   ✓ Comparative result (with vs without hitch control) at 3:52
#   ✓ Limitations explicitly analysed at 4:12
#   ✓ Findings tied back to research question in conclusion
#   ✓ Video production quality: clean cuts, labelled overlays, annotated telemetry


# ==========================================================================
# PRODUCTION NOTES
# ==========================================================================
#
# RECORDING ORDER (suggested):
#   1. Record all simulator footage first — multiple takes for each mode
#   2. Record ArUco/real robot footage (even 10 seconds is sufficient)
#   3. Screen-record paper PDFs with cursor highlighting key equations
#   4. Narrate over assembled footage last — use a script reader at ~130 WPM
#
# SIMULATOR FOOTAGE NEEDED:
#   - forward tracking (figure8 or arc, clean, ~20 sec)
#   - reverse WITHOUT hitch control (jackknife within 5 sec — easy to get)
#   - reverse WITH control, gentle path, hitch in mid-range (figure8, ~30 sec)
#   - demo/hairpin with fixup triggering — slow the video here if needed
#   - telemetry panel close-up showing frustration bar filling and firing
#
# PAPERS TO SHOW ON SCREEN (suggest having PDFs open):
#   - Coulter 1992 (CMU tech report — freely available)
#   - Any kinematic trailer paper (search "trailer kinematics reversing control")
#   - Garrido-Jurado 2014 ArUco (accessible via Google Scholar)
#
# TIMING BUFFER:
#   This script runs approximately 4:58 at 130 WPM narration pace.
#   Trim pauses in simulator footage to fit. Do not rush narration.