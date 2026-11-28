# Grasp verify "no wrist frame" + can regression (2026-11-28)

## Candidate causes (from code), and what the diagnostics said

| candidate | how it could give "no frame" | verdict (smoke_1128c, new diagnostics) |
|---|---|---|
| trigger timing vs sim time | triggered camera renders on its next sensor update; a 3 s WALL window may end before it at RTF < 1 | **no**: every wrist view returned a frame, 0.02-0.05 s after the request (`wrist_views` in attempts.jsonl) |
| camera topic QoS / lazy bridge | frames never reach the node | **no**: bridge not lazy, same subscription works for the servo, frames arrived |
| executor starvation | one mutually-exclusive callback group, 100 Hz joint states | **no**: frames arrived in < 0.1 s |
| render throttling (3 cameras, one render thread) | slow / dropped renders under load | **no** (same evidence) |
| **held object occluding the camera** | at the view pose the held can fills most of the wrist view: < 50 valid depth points or the in-view floor fit fails -> `standing_points` None | **yes**: `wrist_view` OK, `standing_points` None in the 'unconfirmed' try (can_3: 6/6 views ok, lift check None) |

The reply "my wrist camera gave me no picture" was therefore wrong: the picture came, the check could not use it.

## Fix (ef767d5)

`wrist_yaw.standing_check`: uses the floor plane the servo fitted BEFORE the grasp (base frame; the robot has not moved),
so a view mostly filled by the held object still works; and a spot the camera cannot SEE (fewer than 20 floor / object points
within the radius) is UNKNOWN -> None. Before, with any floor, a hidden spot read 0 standing points = "lifted" (a false
confirmation the old code avoided only because its floor fit failed). The attempt saves `standing_why`; the reply says
"my wrist camera couldn't see the spot where it stood". 5 synthetic-depth tests (`gurdy_core/tests/test_standing_check.py`).

## Verification run (smoke_1128d) -- the can broke, so Gazebo work was STOPPED

| run | code | can held by GT | failures |
|---|---|---|---|
| smoke_1128b (04:14-04:28) | before ttl 30 | 3/3 (robot confirmed 1/3) | verify unconfirmed x2 |
| smoke_1128c (05:25-05:47) | ttl 30 + diagnostics | 1/3 | align OCCLUDED ('implausible', can 6.9 cm off-axis); grasp missed x3 (fingers closed on nothing, can knocked over); verify unconfirmed x1 |
| smoke_1128d (05:50-06:11) | + standing_check fix | **0/3** | pre-grasp move failed x3; align OCCLUDED x3; grasp missed x3 (can knocked over) |

None of the c / d failures happen in code changed since b (the verify fix runs after the grasp; ttl only changes how long
an UNCONFIRMED track is kept -- the mapped can position is the same in every run, 1.17-1.20 / -0.06..-0.10). Same Docker images
and colcon install as in b.

**Likely cause (reported by the user afterwards): machine sharing.** Gurdy 4.0's test batches ran on the same laptop / GPU
DURING smoke_1128c and _d: each side started a fresh stack per try and only checked `docker ps` before each try, so the two
batches interleaved in the ~1 s gaps between stacks (and the overlap also broke Gurdy 4.0's results). smoke_1128c/d are
therefore NOT valid measurements of the code; smoke_1128b (no overlap seen) is the last clean can result (3/3 held).
Gazebo work is stopped until a shared GPU lock exists. Re-check the can (x3) and the verify fix only under that lock;
`DETECTOR_TTL` (start_web_robot.sh passthrough) allows a ttl 10 vs 30 A/B in the same session if still needed.

## Re-check under the shared GPU lock (smoke_1128e, 06:23-06:40 UTC, lock held for the whole batch)

| try | can held by GT | robot said | lift check |
|---|---|---|---|
| can 1 | yes (z 0.235 / 0.239 after 5 s), 174 s | "... I can't confirm I'm holding it (my wrist camera couldn't see the spot where it stood) ..." | spot not visible: 0 points within 4 cm (known floor and in-view fit) |
| can 2 | yes (z 0.232 / 0.235), 122 s | same | same; in-view fit: no floor (119 points) |
| can 3 | yes (z 0.236 / 0.240), 296 s | same | same; in-view fit: no floor (159 points) |

Can 3/3 by ground truth again (the c/d failures were the machine sharing). The fingers read 33 / 33 mm (= can width) in all three.
The robot confirmed 0/3: at the verify (view) pose the HELD can sits exactly between the wrist camera and the spot where it
stood, so that spot is never visible -- the check is geometrically blind while holding, and the new code correctly says
"unknown" (the old code read the hidden spot as 0 standing points = "lifted", i.e. earlier confirmations there were not
based on seeing the spot). Consequence: every real pick now ends GRASP_UNCONFIRMED -> a fetch / "bring me" stops after the
pick and keeps holding. Proposed fix (not made): confirm the carried weight from the arm joint efforts (shoulder / elbow
effort with vs without the payload; grasp_test already logs `effort_closed_on_floor` / `effort_hold`, Phase 9i confirmed the
can's weight this way), or a verify pose that looks at the spot from the side, past the held object.
