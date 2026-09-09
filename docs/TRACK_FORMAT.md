# Text track format

Tracks are UTF-8 `.track` files containing one road-building command per line.
Use `#` for comments and optional double quotes for names or descriptions.
Commands are lowercase. Distances are **metres** and angles are **degrees**.
Numbers can be negative or decimal where appropriate; `NaN` and infinity are
rejected. Parse errors identify the source line.

The road begins at `(0, 0, 0)`, pointing along `+Z`. `+Y` is up and `+X` is
initially right. Road width defaults to 12 m. Commands continue from the exact
position, direction, elevation and bank left by the previous command.

## Quick example

```text
name "My Mountain Sprint"
description "A bridge, a tunnel, and a small summit jump."
width 12
start 0 0 0
straight 150
right 60 radius 110 rise 10 bank -12
bridge 100 bank 0
checkpoint
left 60 radius 100 rise 6 bank 12 kind tunnel
straight 80 bank 0
ramp 24 rise 2.5
gap 14 rise -2.5
straight 160
finish
```

Pass the file path to the game with `cargo run --release -- --track tracks/my.track`.
Use `tracks/alpine.track` as a scenic point-to-point example and
`tracks/club.track` as a correctly closed lap circuit. The
[bundled course catalog](../README.md#bundled-courses) also includes focused
examples for width and banking transitions, mountain switchbacks, persistent
road kinds, multiple jumps, and a grade-separated figure-eight crossing.

## Commands

| Command | Meaning |
| --- | --- |
| `name "Name"` | Display name. Quotes are optional for plain text. |
| `description "Text"` | Short course description. |
| `start x y z [heading]` | Initial position and heading, before any road commands. Heading 0 faces +Z; 90 faces +X. |
| `width metres` | Target width for subsequent road. A change blends smoothly across the next road command. |
| `straight length [options]` | Straight road with the supplied horizontal length. |
| `left angle radius metres [options]` | Constant-radius left turn. |
| `right angle radius metres [options]` | Constant-radius right turn. |
| `bridge length [options]` | Straight elevated road with bridge styling/supports. |
| `tunnel length [options]` | Straight covered road with tunnel styling. |
| `ramp length rise height [options]` | Straight rising road that retains an uphill slope at its exit. |
| `gap length [options]` | Missing road; retains a route guide for progress and landing alignment. |
| `kind road\|bridge\|tunnel\|ramp\|gap` | Default kind for subsequent `straight`, `left` and `right` commands. |
| `checkpoint` | Place an ordered gate at the current road endpoint. |
| `finish` | End a point-to-point course. Also implied by end of file. |
| `close` | End a lap course after validating that its endpoint matches its start. |

Each road-building command accepts these optional pairs in any order:

| Option | Meaning |
| --- | --- |
| `rise height` | Elevation change over this command; defaults to 0. Negative values descend. |
| `bank angle` | Target banking angle at the end of this command; defaults to the current bank. |
| `kind type` | Override this command's road kind, including on a curve. |

An explicit `kind` option applies to that command only. Likewise, `bridge 80`
creates one bridge segment and does not change the default kind. To make a
curved bridge, write `right 45 radius 100 kind bridge`, or use `kind bridge`
before several commands and `kind road` afterward.

## Hills, banking and jumps

Ordinary elevation changes use a smooth cubic profile: they preserve the
incoming slope and finish level. Consecutive rises therefore form eased climbs
with level command boundaries. Use longer commands for gentler hills. A
`straight` after a ramp preserves the incoming slope at the join before easing
toward its target elevation.

Bank angles blend smoothly from the current bank to the target bank across the
command. **Positive bank raises the right road edge**, helping a left turn.
Use **negative bank for right turns**. Banking persists until another `bank`
option changes it; `bank 0` returns the road to level. Split a long turn into
entry, middle and exit commands when you want bank to build early, hold steady,
and unwind near the exit.

A ramp needs positive `rise`. Its cubic elevation profile leaves the road with
an uphill grade equal to 1.5 times `rise / length`, allowing the car to launch.
Follow it with a `gap` for an actual break in the surface. The gap's `rise`
positions the landing relative to the ramp lip; it does not create a physical
surface. Its invisible centerline eases from the launch slope to a level
landing. The car's actual flight is determined by its velocity and gravity.
For example, `ramp 24 rise 2.5`, `gap 14 rise -2.5`, `straight 100` makes a short
jump landing back at the pre-ramp elevation. Test jump length at the intended
approach speed. A track may not start or finish in a gap.

Bridges and tunnels use the same road geometry and physics as normal roads;
their road kind instructs the renderer to add their supporting structure.
Use elevation commands to raise bridges above lower roads or the landscape.
The base landscape automatically sits 3 m below the course's lowest road edge,
including its banking. Negative starting elevations therefore remain above
their terrain; world height zero has no special surface or ground-plane role.
Tracks can cross their own paths at different elevations. Authors are
responsible for enough vertical clearance and for avoiding accidental surface
overlap; the parser does not check all-world intersections.

## Time trials and closed circuits

Sprint timing and the finish gate sit 3 m before the road endpoint. That
position must be on solid road; extend the landing if a final gap would cover
it. Circuits use a single start/finish gate at the closing seam.

Checkpoints must be driven in order. Start and finish are implicit; place
interior checkpoints at useful road boundaries. They must be at least 10 m
from the start, finish, and one another. Do not place a checkpoint at the start
or end of a gap; put it on solid road before the launch or after a landing road.
When none are supplied, the parser inserts gates at one-quarter,
one-half and three-quarters of the route, skipping gaps and their boundaries
and keeping the same 10 m clearance from the start and finish.

`close` does **not** invent a connector or teleport the road. Before it, build
back to within 0.25 m of the start with matching heading, slope, bank and width.
The parser removes tiny rounding drift at the seam, provided this does not
collapse or reverse the final sampled segment or make its slope exceed 100%.
A short, densely sampled final transition may need a more precise endpoint
or a longer final command.
This complete oval closes:

```text
straight 140
right 180 radius 55
straight 140
right 180 radius 55
close
```

## Limits and authoring tips

- Each command spans 1–5000 horizontal metres; turn angles are 0.1–360 degrees.
- Radius is 8–5000 m and must exceed half the road width plus 1 m.
- Width is 4–40 m; banking is between -60 and +60 degrees.
- A rise is between -500 and +500 m and no more than 60% of horizontal length.
  Slopes throughout the cubic elevation profile may not exceed 100%, including
  between generated samples; use longer transitions if rejected.
- Road elevation stays between -1000 and +2000 m, including peaks between
  generated samples. Start coordinates are
  limited to ±10000 m horizontally and ±1000 m vertically.
- A track must be at least 20 m long, at most 50 km, with at most 25001 samples.
  Track files are limited to 1 MB; name and description each to 500 bytes.
- Roads are sampled about every 2 m (more densely on elevation and banking
  transitions).
  Even short elevation transitions include interior samples so their hills
  and incoming ramp slopes remain represented in the driving surface.
  Short banking changes receive extra samples to keep the rendered edges close
  to the full-width driving surface.
  Timing distances use the resulting **3D centerline distance**, so an uphill
  `straight 100` can contribute slightly more than 100 m to course length.
- Put a straight after the final corner to allow braking before a sprint ends.
- `#` inside quoted text is preserved. Inside quotes, `\"` and `\\` are the
  supported escape sequences. Unknown commands/options and duplicate options
  are errors, helping catch typos early.

## Rust interface

`Track::parse(&str)` and `Track::load(path)` return `Result<Track, String>`.
`Track::sample_at(distance)` interpolates position and an orthonormal road
frame, clamps point-to-point routes and wraps closed routes in either
direction. Non-finite lookup distances return the starting sample.
`Track::ground_height()` returns the base terrain elevation, 3 m below the
lowest sampled banked road edge.
`Track::finish_distance()` returns the distance used by timing and the finish
gate: the course length for circuits, or 3 m before the endpoint for sprints.
`Track::source_hash()` identifies the exact source bytes used to build the
loaded route, so record files stay associated with that version until reload.

Each `RoadSample` contains `pos`, `forward`, `right`, `up`, `width`, `bank`
(radians), cumulative `distance`, and `kind`. Its `kind` describes the
**outgoing** segment to the next sample. A gap starts exactly at its first
sample and ends exactly at the first following non-gap sample. Use this rule
consistently for road meshes and collision queries.
