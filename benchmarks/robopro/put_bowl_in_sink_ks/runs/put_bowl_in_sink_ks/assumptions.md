# Assumptions of the put_bowl_in_sink_ks pipeline

Line numbers refer to `pipeline.py` as committed for Phase 1 (1c38fb2). Each assumption names the code that
relies on it and how likely a real kitchen is to violate it.

| # | Assumption | Relied on at | Real-world risk |
|---|---|---|---|
| A1 | The basin is empty: its middle is free down to the floor, so a bowl released 3 cm above the floor at the middle lands on the floor. | 57 (target = sink centre), 63 (release height from the basin floor) | High: sinks usually hold dishes, a pot or a colander. |
| A2 | The bowl stands upright on the counter, so its rim can be pinched from above. | 49 (`strategy="top"`) | Medium: bowls are left upside down to drain, or on their side. |
| A3 | The space straight above the bowl and the straight line from there to above the sink are free; the planner does not see the held bowl, so nothing may stand in its way. | 53 (lift), 57-59 (straight carry) | Medium: bottles, a soap dispenser or a faucet between bowl and sink. |
| A4 | Lifting the bowl's bottom 5 cm above the counter clears everything on the way (the sink rim, other items). | 53 (`CLEAR`) | Medium: tall items on the counter. |
| A5 | The bowl does not move between detection and grasp, nor during the carry. | 45, 49 (one detection before the grasp) | Low-medium: bowls slide when bumped. |
| A6 | A pinch on the rim holds the bowl through the carry: it is light (0.05 kg) and not slippery. | 49, 53-63 (no hold check after the grasp) | Medium: a bowl with food or water in it is several times heavier; wet ceramic is slippery. |
| A7 | The bowl's box (from detection) gives its lowest point, also when it tilts in the pinch. | 52-53, 61-63 (`bottom(b)`) | Low (detection is pose-based; noise 1 cm, averaged). |
| A8 | The basin's floor is level and at the bottom of the sink's box; the basin's middle is the best release point. | 46, 57, 63 | Medium: sinks have a drain, a divider, a grid. |
| A9 | Opening the gripper and going straight up leaves the bowl in place: the fingers straddle the rim wall along the approach. | 65-66 | Low. |
| A10 | The bowl lies on the right half of the counter (x > 0), within reach of the arm that picks it. | 47 | Low here (spawn region), higher in general. |
| A11 | The sink is where its detection says, and nothing about it changes during the episode (no water running, nothing dropped into it). | 44, 57, 63 | Medium: the scene can change while the robot carries the bowl. |
| A12 | Appearance does not matter: perception is pose-based. | all `detect` calls | Not attackable in this backend (colour and lighting do not reach detection). |
| A13 | The other arm is out of the way and stays where it is. | (implicit) | Low. |

Most likely to be violated first: **A1** (something already in the sink).
