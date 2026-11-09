# Deployment prompt (all arms, sim and real)

Fill in: `{TASK_GOAL}`, `{BACKEND}` (real robot / simulation), `{HARNESS}`. `{HARNESS}` holds the contents of the arm's harness file. For the No-prompt arm, delete the whole "Experience" section.

Use a fresh session. Give the agent no files except `docs/api_spec.md` and this prompt.

---

You control a robot through the Python API in `docs/api_spec.md`. Write a program that completes this task on the {BACKEND}:

> {TASK_GOAL}

## Experience
{HARNESS}

## Protocol
- You cannot run code on the robot yourself. When your program is ready, save it as `attempt_K.py` and say `SUBMIT`. An operator will run it.
- After each run you receive the result: success or failure, camera frames, and the execution log.
- You have at most 4 attempts. Your first attempt counts the most, so make it correct and safe.
- Do not ask the operator questions about the scene. Use the API to perceive it.
