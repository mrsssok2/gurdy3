# Demo runbook: "bring me the red can" (Gazebo, lab4_fetch)

Branch `phase9k-demo` (2026-11-27, CPU-only prep). **Nothing in this runbook has been run yet**: the new delivery path
(`--fetch-bring-me`) is tested on the fake robot only (`brain/tests/test_fetch_bring_me.py`). Do one rehearsal before
recording. Times below are from earlier Gazebo runs where they were measured, otherwise marked *not measured*.

## 0. Before you start

```bash
cd ~/projects/gurdy3
git log --oneline -1                 # phase9k-demo must be merged (section 7) before recording
docker ps                            # MUST be empty (one heavy stack; Gurdy 4.0 / the overnight run must be finished)
```

Close the browser tabs that hold an old token. Keep the terminal **off the recording**: it prints the access token
(`READY ... #token=...`), and so does `/tmp/gurdy_web.log`.

## 1. Start the stack (Gazebo window + web app + arm + MoveIt)

```bash
GUI=true MODEL=keyword FETCH=true FETCH_EXTRA="--fetch-bring-me --fetch-gt log" scripts/start_web_robot.sh
```

* `FETCH=true`: world `lab4_fetch`, arm + MoveIt (`sim_moveit`), detector det_v3_s, agent `--fetch`.
* `--fetch-bring-me`: the robot comes back to where the request came from, opens the gripper only when it is standing
  still, and says "Here's your red can." only when the state and the Gazebo ground truth confirm the delivery.
* `--fetch-gt log`: Gazebo ground truth is also written next to every grasp attempt (diagnostics; the grasp uses the
  detector + wrist camera, not GT).
* `MODEL=keyword`: no LLM / Ollama needed; "bring me the red can" goes to the fetch router anyway. Use
  `MODEL=gurdy-sft-400` only if other chat should look smart too (needs Ollama, see CLAUDE.md).
* The start takes ~2.5 min (start_web_robot.sh's own estimate). Wait for `READY`. Check the agent log says
  `fetch bring-me delivery: ON (ground truth yes)` and `fetch: ON (MoveIt yes)`:

```bash
grep -E "fetch bring-me|fetch: ON" /tmp/gurdy_web.log
```

## 2. Stage the scene (red can 1.3 m in front of the robot)

At start the red can stands in room C, 3.2 m away and out of view, so the robot would first need a whole-house search
(*not measured* end to end with the map-sized limit). For the recording, put it 1.3 m straight ahead of the spawn point
(it's in the camera view at start; (1.3, 0.0) passes the same free-space test as the overnight plan, 12 of 16 approach
headings free, > 0.8 m from doorways):

```bash
docker exec gurdy3_agent /entrypoint.sh gz service -s /world/lab4_fetch/set_pose --reqtype gz.msgs.Pose \
  --reptype gz.msgs.Boolean --timeout 3000 --req 'name: "f_red_can" position {x: 1.3 y: 0.0 z: 0.0} orientation {w: 1}'
```

Expect `data: true`. Wait ~10 s: the detector confirms a track after >= 8 hits over >= 2 s. In the web map the red can
should appear in front of the robot. If it doesn't, the robot will search (that still works, it just takes longer).

Optional, for a harder take: skip this step (red can in room C, search first).

## 3. Open the dashboard without showing the token

1. Copy the `http://localhost:8080/#token=...` URL from the terminal **before** recording.
2. Paste it into the browser. The page stores the token and removes it from the address bar (`history.replaceState`).
3. Press **F11** (full screen: no address bar, no tabs). Never press Ctrl+L / F6 on camera.
4. The browser history still holds the URL: don't open the history or the new-tab page while recording.

## 4. Camera angles (3 sources, cut together)

| shot | source | framing | when |
|---|---|---|---|
| A. wide | Gazebo window | from behind and to the left of the spawn point, ~1.6 m up, looking at (0.8, 0): robot + can + hand-over spot in one frame | whole take |
| B. close | Gazebo window | low, from the side of the can at (1.3, 0), ~0.6 m away: the wrist camera approach, the grasp and the lift | approach -> lift |
| C. dashboard | browser (F11) | full screen: map (robot trail, can marker), camera card, chat with the command and the reply | command, reply |

Gazebo GUI camera: right-click the `gurdy` model -> **Move to**, then orbit (left drag), pan (middle drag), zoom
(wheel). For shot B, right-click `f_red_can` -> Move to. A **Follow** on `gurdy` keeps the robot centred during the carry
(right-click -> Follow; Esc / "Free camera" to stop). Gazebo window full screen: F11 inside it too, or maximise it.

Record: Windows Game Bar (Win + Alt + R records the focused window only), or OBS with two window captures (Gazebo,
browser) as scenes, 1920x1080, 30 fps. WSLg windows show up as normal Windows windows.

## 5. The take

1. Start recording (shot A).
2. In the dashboard chat, type: **`bring me the red can`** (Enter). Or the mic button if started with `voice`
   (`... scripts/start_web_robot.sh voice`; localhost counts as a secure origin, no https needed on the laptop).
3. The robot, without stopping: plans to a stand-off pose facing the can, turns round (the arm is at the back), backs
   up, arm to the view pose, wrist-camera servo, grasp, lift, checks nothing is still standing at the old spot, folds the
   arm, carries it back at <= 0.08 m/s to 0.6 m from where it started, turns its back to you, presents it at ~15 cm,
   waits until its body is still (|v| < 0.01 m/s for 1 s), opens, folds.
4. Reply in the chat: **"Here's your red can."** Only if the release was confirmed. Any other reply is honest and tells
   you what happened (see section 6).
5. Stop recording.

Duration: *not measured* for this exact path. Earlier Gazebo numbers: fetch retest C1 (search + Nav2 + detector +
grasp, pick only) 253 s; carry at 0.08 m/s over ~1 m is ~15-30 s plus turning. Plan for 5-8 minutes per take.

Check after the take (ground truth: where the can really is):

```bash
docker exec gurdy3_agent /entrypoint.sh timeout 8 gz topic -e -n 1 -t /world/lab4_fetch/pose/info | grep -A8 '"f_red_can"' | head -9
grep -E '"deliver"' /tmp/gurdy_web.log | tail -3      # the robot's own release record (fingers before / after, GT after)
```

Reset for another take: Ctrl+C the stack and start again (section 1). A fresh stack per take was the rule in all the
fetch runs; re-staging the can with set_pose works too, but the map / memory still holds the old track.

## 6. Replies you may see and what they mean

| reply | meaning | what to do |
|---|---|---|
| Here's your red can. | released while standing still, fingers opened, GT shows the can at the release point | keep the take |
| I dropped the red can on the way. It's on the floor in room A ... | fingers closed, or GT showed the can left the gripper during the carry | re-stage, new take |
| I couldn't stand still to hand over the red can, so I didn't let go ... | the body kept moving for 15 s at the hand-over pose | "hand it to me" again |
| My gripper did not open, so I'm still holding the red can. | fingers did not open wider than while holding | restart the stack |
| I opened my gripper, but I can't confirm that the red can reached you ... | opened, but GT does not show the can at the release point | check the GT command above |
| I couldn't get a grip on the red can after 3 attempts ... | grasp failed 3 times (fetch retest: the main failure mode) | re-stage, new take |
| I stopped (emergency stop). ... | E-STOP pressed | Release, new take |

## 7. Merge for tomorrow (after the overnight run has committed and pushed)

```bash
cd ~/projects/gurdy3
docker ps                                           # empty: the overnight run is finished
git status --short                                  # nothing of the overnight run left uncommitted
git pull --no-rebase                                # the overnight results commit
git merge --no-ff phase9k-demo -m "Merge phase9k-demo: bring-me delivery (flag off), map-sized fetch search, keyword counts/schedules"
PYTHONPATH=gurdy_core ~/projects/.venv-gurdy3/bin/python -m pytest -q brain/tests   # host check (~2 min)
git push
git worktree remove ~/projects/gurdy3-demo          # optional, once merged
```

Known limits (not fixed):
* Without Gazebo ground truth (a real robot), the delivery check is the fingers only: a can that fell out while the
  fingers stayed jammed at its width would be reported as delivered (pinned by
  `test_known_limit_state_only_cannot_see_a_jammed_silent_drop`). In Gazebo the GT check catches it.
* The requester pose is the robot pose when the command arrives (or a saved place `me` / `user` / `requester`). The robot
  doesn't know where a person stands; for the demo, the "person" is the spot where the robot started.
* The release drops the can from ~15 cm onto the floor behind the robot (no hand in the simulation); a cylinder can roll.
  The GT check allows 0.15 m.
