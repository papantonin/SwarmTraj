# Historical contact regression

`historical-contact-seed19.npz` contains one rejected geometric/Python proposal
from revision `300de37`, not an accepted mission or a performance result.
It preserves the original negative regression after removing that generator.

The fixture uses seed 19, drone 3, candidate 0, the four-aircraft separate loiter
layout at 30/55/80/105 m, target `(500, 500, 30)`, 180 s horizon, 30° descent,
40–110 m cruise levels, 180° approach spread and `static_obstacles.json`.
Its independently reintegrated first contact occurs at 81.47613754272462 s with
31.01777955484954° descent, outside the unchanged 1° tolerance. Independent
validation must continue rejecting it as `validation_descent_angle`.

The NPZ uses numeric arrays and a JSON metadata string; loading requires no
pickled objects. No historical proposal implementation is shipped at runtime.
