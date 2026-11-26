"""Phase 9d: fetch commands ("bring me the red can", "put the mini toolbox in room B", "hand it to me").

Same pattern as brain.incidents (Phase 6): the fetch tools are NOT in brain.tools.TOOLS (the fine-tuned planner never saw
them); utterances are routed deterministically (router.py), validated against strict specs (specs.py), checked by the
fetch safety rules (safety.py), executed by a task state machine (machine.py) over a FetchBackend (mock.py: mocked robot
only), and answered from templates built from the machine's CONFIRMED outcome (replies.py + its grounding check).
Stdlib + PyYAML only.
"""
