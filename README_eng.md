# BetterPenetrationNext 1.0.9 — KKS

[Русский](README_rus.md) · [Download](https://github.com/bibemx/BetterPenetrationNext/releases)

A reactive rig and local animated-mesh deformation module for Koikatsu Sunshine / Unity 2019.4. The project studies geometric constraints, deformable shells and skeletal animation. It is an approximate visual model; geometric tests do not constitute clinical validation.

## Features

- **BPN** mode: Adaptive Mesh Landmarks, an oriented entrance plane R0 and a channel guide.
- An attached Source base, preserved section lengths, and manual length/width multipliers applied as soon as the scene loads.
- Independent Receiver sectors fitted to finite cuts of the actual open Source shell. A cached BVH is refitted without rebuilding topology or merging differently weighted seam identities.
- A smooth terminal approach and bounded local cervical compression. Contact does not move the uterus or extend the channel; baseline channel length is a separate setting.
- Entrance attachments, symmetric procedural displacements and smooth recovery after contact ends. The tested recovery setting is 4.5 seconds, a visualization parameter.
- Textured X-Ray, separate Source/Receiver rendering and an optional dot indicator, without overwriting original materials.
- **BP** mode runs the retained BetterPenetration controller inside this DLL. A separate original BP DLL must not run alongside it.

## Requirements

| Component | Requirement |
|---|---|
| Game | Koikatsu Sunshine, Unity **2019.4.9f1**, Windows x64 |
| Loader | [BepInEx 5.x](https://github.com/BepInEx/BepInEx/releases), including `0Harmony.dll` |
| API | [IllusionModdingAPI / KKSAPI](https://github.com/IllusionMods/IllusionModdingAPI/releases) |
| ZIPMOD loading | Sideloader from [BepisPlugins](https://github.com/IllusionMods/BepisPlugins/releases) |
| Model selection | [UncensorSelector](https://github.com/IllusionMods/KK_Plugins/releases), plugin version **3.11.1** or later |
| Model bones | [ModBoneImplantor](https://github.com/IllusionMods/ModBoneImplantor/releases), plugin version **1.1.1** or later |
| Source | Tested with `[KKS][Penis][BP] SoS.zipmod`, GUID `uncensor.BP_SoS`, selection `BP SoS` |
| Receiver | `BPN.Reciever.Profundis.zipmod` included in this release |

Game libraries, the original SoS asset and dependency plugins are not bundled. Obtain compatible original assets from [upstream BetterPenetration](https://github.com/IllusionMods/BetterPenetration/releases). Studio and other games are not supported by this release. ConfigurationManager is optional because BPN provides its own settings window.

## Installation

1. Install the dependencies and Source SoS. Close KKS and Studio.
2. Download **`BetterPenetrationNext-1.0.9-KKS.zip`** from Releases and extract it to a separate directory. GitHub's automatic `Source code.zip` is not an installation package.
3. Run `Install.cmd` and enter the directory containing `KoikatsuSunshine.exe`, or use:

   ```powershell
   powershell -NoProfile -ExecutionPolicy Bypass -File .\Install.ps1 -GameRoot "D:\Games\KKS"
   ```

4. The installer validates the package, backs up affected files, installs the DLL and Receiver, and applies the tested defaults. Active legacy `BetterPenetrationNext-1.0.zipmod` and detected original/Enhanced BP DLLs are disabled by renaming with backups. Already disabled files are preserved; upgrading a BPN test build is supported.
5. In UncensorSelector, choose **BetterPenetration Next 1.0** for the Receiver and **BP SoS** for the Source. Remove other active Receiver copies using GUID `org.betterpenetration.next.body.v1`.
6. Load the scene. Press **B** for X-Ray and **N** for settings. Both shortcuts can be rebound under `Next.Controls`. The current settings window uses Russian labels.

The installer does not modify character cards, user textures or the Source ZIPMOD. Never overwrite the DLL while the game is running. Use the **BPN / BP** switch for the original controller; do not run its original DLL concurrently.

## Defaults

The same `release-defaults.json` is used by the installer and embedded in the DLL. A clean configuration, or deleting `BepInEx/config/org.betterpenetration.next.cfg`, recreates these defaults. Only delete it with the game closed and after saving your own backup.

| Tested setting | Value |
|---|---:|
| SourceLengthScale | 0.7727273 |
| SourceWidthScale | 1 |
| CanalLengthScale | 1.384297 |
| ContactDamping | 5.419422 |
| CervixMaxDisplacement | 0.08 |
| CervixFollowStrength | 1 |
| RecoveryTime | 4.5 |
| X-Ray / settings | B / N |

Dimensions are relative to the animated card model. These values were tested on one specific model pair and are not universal anatomical standards. The window exposes the current reactive rig settings; compatible internal legacy parameters remain in code but are hidden from the main UI. The prepared Receiver does not require an external `PathFile`.

## Rollback

Close the game and run `Rollback.ps1` **from the backup directory** printed by the installer:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File "D:\Games\KKS\BepInEx\BetterPenetrationNextBackups\1.0.9-...\Rollback.ps1"
```

This restores the previous DLLs, Receiver and configuration, including previously active original files. Files changed after installation are saved again before restoration. Backups are retained.

## Build and tests

Use a .NET SDK supporting `net6.0` and `net46` projects (tested: **6.0.402**) and a local KKS installation with the dependencies. The build restores Microsoft's .NET 4.6 reference assembly package. Game DLLs are not redistributed in this repository.

```powershell
dotnet build src/Next.Plugin -c Release -p:GameRoot="D:\Games\KKS" -m:1
dotnet run --project tests/Next.Core.Tests -c Release
```

Output: `src/Next.Plugin/bin/Release/net46/BetterPenetrationNext.dll`. Core tests run without the game. `tests/AnatomicalRuntimeReplay` retains the replay tool; private scene captures and external source models are not committed. The prebuilt shader bundle is under `build/shader`; shader source is under `UnityLab/Assets/Shaders`.

Validation includes 173 core tests, native real-mesh checks in Unity and user acceptance of an active scene. One accepted scene does not guarantee intersection-free results for every card and animation. Full-frame timing depends on scene and background load; a guaranteed FPS improvement is not claimed. See [docs/validation.md](docs/validation.md) for scope.

## License

Code: [GPL-3.0-or-later](LICENSE). Original BetterPenetration and model credits are retained in [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) and ZIPMOD manifests. The code license does not replace third-party game asset terms.
