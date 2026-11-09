# Summarize prompt (GAH and RSI use the same one)

Fill in: `{REFLECTION_FILES}`, `{OUT}` (`harness/harness_GAH_vX.md` or `harness/harness_RSI.md`).

---

Below are reflections from developing robot programs on several simulated tasks: {REFLECTION_FILES}.

Write `{OUT}`, a harness that a **fresh** coding agent will read before solving a **different** manipulation task on a **real robot**. That agent has no simulator. It must get the program right on the first attempt and has at most 4 attempts.

Requirements:
1. **Content:**
   - Use only lessons supported by the reflections. Do not add generic advice that no reflection supports.
   - Do not name any training task or object.
   - Merge duplicate rules.
   - Order rules by how many failures they would have prevented.
   - Each rule gets one line of reason that cites the kind of failure it prevents.
2. **Structure.** Use only the sections the lessons support:
   - **Before writing code**: questions to ask about the scene and the program's assumptions.
   - **Constraints**: properties every program must have.
   - **Self-checks without a simulator**: checks the program performs at run time, such as re-detecting before grasping, verifying after each step, and timeouts.
   - **Workflow**: the order in which to plan, write, review, and submit.
3. **Length:** at most 800 words.
4. **Source map:** append an HTML comment at the end that maps each rule to the reflection IDs it came from. The deployed agent will ignore it, and we use it for analysis.
