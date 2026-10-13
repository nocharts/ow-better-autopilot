![Better Autopilot](cover.png)

# Better Autopilot

A faster, smarter ship autopilot for Outer Wilds.

## Features

- **Faster flights.** The ship's thrusters are limited per direction, so firing two or three at once pushes harder. The autopilot does exactly that, and turns the ship to make the most of it.
- **Routes around planets and the Sun.** It predicts where planets and moons will be and plans a path around them, instead of flying straight through whatever is in the way.
- **Precise arrivals.** Braking is planned for the thrust and gravity you actually have, so the ship stops where it should instead of overshooting.
- **Clean take-offs.** Starting near a surface, the ship lifts straight up before heading out.
- **Nicer view.** Keeps the destination ahead of you while speeding up, and turns to face it when you arrive.
- **Game default mode.** Switch back to the original autopilot at any time.

## Controls

In the ship's cockpit:

| Action | Keyboard | Controller |
|---|---|---|
| Switch autopilot mode (Game default / Fast / Fastest) | J | D-pad Right |
| Turn auto-align on or off (during autopilot) | G | D-pad Left |

The controls also show up in the cockpit's on-screen prompts, and can be changed in the mod settings.

## Settings

Everything is adjustable in the mod manager's settings for this mod: autopilot mode, auto-align, keeping the destination in view, facing the destination on arrival, avoiding planets and the Sun, faster Match Velocity, a speed limit, braking strength and the controls.

## Installation

Install with the [Outer Wilds Mod Manager](https://outerwildsmods.com/mod-manager/).

Works on Outer Wilds 1.1.16 with OWML 2.16. Works alongside [Landing Assist](https://github.com/nocharts/ow-landing-assist).

## Building

Requires the .NET SDK. `dotnet build BetterAutopilot/BetterAutopilot.csproj -c Release` builds the mod; if the Outer Wilds Mod Manager is installed (Flatpak), the build also copies it into the manager's mods folder.

## License

[MIT](LICENSE)
