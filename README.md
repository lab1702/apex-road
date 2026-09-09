# APEX / ROAD

A single-player 3D time-trial racer written in Rust. The whole window is your
windshield: a clean, stylized world ahead, with a floating gauge cluster at the
bottom. Build your own roads with a text file, then chase a better time.

The handling sits between arcade and simulation. Keyboard inputs are smoothed
so the car is approachable, while acceleration, braking, and cornering share a
limited amount of tire grip. Enter a corner too quickly, brake too hard, or
apply too much throttle while turning and the tires start to slide. Hills,
banked corners, and airborne jumps affect how the car moves.

## Run on Linux

Install a current stable Rust toolchain and a C linker. The renderer uses
Macroquad and OpenGL; an ordinary graphical Linux desktop is required to play.
On Debian or Ubuntu, the development packages are:

```sh
sudo apt install build-essential pkg-config libx11-dev libxi-dev libgl1-mesa-dev
```

Clone the repository and run:

```sh
git clone https://github.com/lab1702/apex-road.git
cd apex-road
cargo run --release
```

Press **Enter** to start. The first build downloads and compiles the Rust
dependencies; subsequent starts are faster. Run from the project directory so
the game can find the bundled `tracks/` folder.

## Drive

| Key | Action |
| --- | --- |
| Enter | Start a run; resume when paused |
| W / Up | Accelerate |
| S / Down | Brake; hold for 0.5 seconds when stopped to reverse |
| A / D or Left / Right | Steer |
| Space | Handbrake |
| R | Restart the current time trial |
| Esc | Pause / resume |
| F1 | Show / hide the control guide |
| F5 | Reload the current track file |
| Tab | Switch to the next bundled track |
| F11 | Toggle fullscreen |
| Q, while paused | Quit |

Brake before a tight corner, ease off as you turn, and feed the throttle back
in on the exit. The orange traction indicator shows when the car is losing
grip; the throttle, brake, steering, and engine indicators show your inputs.
Use the handbrake sparingly—it makes the rear tires easier to slide.

Pass the course checkpoints **in order**, then cross the finish. The gauge
cluster stays low in the windshield, with the timer, checkpoint progress, and
course map above. A run that misses a checkpoint cannot set a record.

Best times are saved under `./data/`, keyed by the track file's contents.
Editing a track starts a separate record history for that version. Restarting
a run resets its timer and checkpoint progress.

## Bundled courses

Press **Tab** to cycle through all seven courses in this order. Each text file
has commented sections you can reuse when building your own tracks.

| Course | Length | Format | Road-building features |
| --- | --- | --- | --- |
| [Alpine Run](tracks/alpine.track) | 1.98 km | Sprint | Hills, banking, bridge, curved tunnel, one jump |
| [Club Circuit](tracks/club.track) | 0.63 km | Circuit | Simple oval, banked semicircles, exact loop closure |
| [Camber Loop](tracks/camber_loop.track) | 1.22 km | Circuit | Split banking transitions, changing widths, raised causeway |
| [Ridgeway Pass](tracks/ridgeway.track) | 1.32 km | Sprint | Mountain climb, downhill switchbacks, narrowing road |
| [Stone Gallery](tracks/stone_gallery.track) | 1.65 km | Sprint | Curved galleries and viaducts, persistent road kinds, negative starting elevation |
| [Airfield Run](tracks/airfield.track) | 1.38 km | Sprint | Three progressively longer jumps, broad landings, elevated bridge |
| [Skyline Eight](tracks/skyline_eight.track) | 1.09 km | Circuit | Figure-eight layout with a 24 m high crossing, banked climbing and descending loops |

Choose a track at launch:

```sh
cargo run --release -- --track tracks/ridgeway.track
```

Or watch the auto driver:

```sh
cargo run --release -- --track tracks/skyline_eight.track --autodrive
```

Auto-drive starts immediately and stays enabled when switching tracks with
**Tab** or reloading with **F5**. Circuits run consecutive laps; sprints stop at
the finish. Press **R** to run a sprint again. Auto-drive does not save records.

## Build a track in text

A `.track` file contains one road-building command per line. Lengths are in
meters and angles are in degrees. For example:

```text
name "My First Sprint"
description "A climb, a banked turn, and a bridge."
width 12
straight 150
right 60 radius 110 rise 10 bank -12
bridge 100 bank 0
checkpoint
left 60 radius 100 kind tunnel
straight 160
finish
```

Save this as `tracks/my.track`, validate it, then drive it:

```sh
cargo run --release -- --validate tracks/my.track
cargo run --release -- --track tracks/my.track
```

While editing a loaded track, press **F5** to reload it. A malformed edit
reports an error and leaves the currently loaded track available. A successful
reload rebuilds the world and restarts the run.

The format supports straights, left and right turns, elevation changes,
banking, width changes, bridges, tunnels, launch ramps, gaps, and closed
circuits. Elevated road crossings can form overpasses; you control their
clearance. See [`docs/TRACK_FORMAT.md`](docs/TRACK_FORMAT.md) for the complete
grammar, examples, limits, and track-design notes.

## Validation and preview tools

| Argument | Purpose |
| --- | --- |
| `--track PATH` | Load a specific `.track` file |
| `--validate PATH` | Parse and check a track without opening a window |
| `--smoke-test` | Run a 240-frame automated driving demo, then exit |
| `--autodrive` | Enable automated driving for a graphics demo |
| `--frames N` | Exit after a specified number of rendered frames |
| `--at METERS` | Begin the preview at a distance along the track |
| `--capture PATH` | Write a PNG screenshot |

Validate example bundled tracks without a display:

```sh
cargo run --release -- --validate tracks/alpine.track
cargo run --release -- --validate tracks/club.track
```

Exercise the renderer and save a driving screenshot:

```sh
cargo run --release -- --smoke-test --capture captures/drive.png
```

Inspect another part of a course:

```sh
cargo run --release -- --track tracks/alpine.track --at 600 --autodrive --frames 120 --capture captures/preview.png
```

Graphics previews need a working OpenGL display, just like normal play.
`--validate` is headless. Automated driving is a development aid for inspecting
the world and renderer, not a racing opponent.

Run the parser, vehicle, and timing checks with:

```sh
cargo test
```

## How it is built

- **Rust + Macroquad:** native window, input, 3D meshes, custom fog shader, and
  2D instruments.
- **Procedural world:** the text track is sampled into a 3D road frame shared
  by rendering and road contact. Scenery and architecture are generated in
  code, with no external art assets needed.
- **Custom vehicle model:** a fixed 120 Hz simulation with combined tire
  traction, speed-sensitive keyboard steering, weight transfer effects, road
  banking, and airborne motion.
- **Time trials:** ordered checkpoints, finish timing, and persistent local
  records tied to track contents.

This first version has one car, keyboard controls, dry weather, and procedural
visuals. Audio is not included in this first version. It is a focused solo racer,
with simplified vehicle physics rather than a full mechanical or tire simulation. Tracks are authored in text;
there is no graphical editor. Authors should test jumps, road crossings, and
clearances at their intended driving speeds.

## License

The game is licensed under the [MIT License](LICENSE).
The bundled DejaVu Sans font retains its [own license and attribution](assets/DejaVuSans-LICENSE.txt).
