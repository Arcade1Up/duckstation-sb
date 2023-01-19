- 2021/07/25: Ability to boot games directly from CD-ROM added. You may want to reduce the readahead size to reduce hitches on seek/loading.
- 2021/07/11: UWP/Xbox one port added. Follow the instructions in "Universal Windows Platform / Xbox One" below.
- 2021/07/10: Direct3D 12 hardware renderer added. It does not support downsampling or postprocessing (was mainly intended for Xbox).
- 2021/06/25: Ability to undelete files from memory card editor added.
- 2021/06/22: Measured achievements for RetroAchievements added.
- 2021/06/19: Leaderboards for RetroAchievements added.
- 2021/06/01: Auto loading/applying of PPF patches added.
- 2021/05/23: Save RAM (srm) support added to libretro core.
- 2021/05/23: CD-ROM seek speedup enhancement added.
- 2021/05/16: Auto fire (toggle pressing) buttons added.
- 2021/05/10: libretro core re-added. Commercial distribution of the DuckStation libretro core is **NOT PERMITTED**.
- 2021/05/02: New pause menu added to Android app.
- 2021/04/29: Custom aspect ratio support added.
- 2021/03/20: Memory card editor added to Android app.
- 2021/03/17: Add support for loading **homebrew** PBP images. PSN images are not loadable due to potential legal issues surrounding the encryption.
- 2021/03/14: Multiple controllers, multitap, and external controller vibration added to Android app. You will need to rebind your controllers.
- 2021/03/14: RetroAchievements added to Android app.
- 2021/03/03: RetroAchievements are now available. You can now log in to your retroacheivements.org account in DuckStation and gain points in supported games. Currently only for Windows/Linux/Mac, Android will be added in the future.
- 2021/03/03: Multitap is now supported for up to 8 controllers. You can choose which of the two main controller ports have taps connected in Console Settings and bind controllers in Controller Settings.
- 2021/03/03: Ability to add/remove touchscreen controller buttons and change opacity added for the Android app.
- 2021/01/31: "Fullscreen UI" added, aka "Big Duck/TV Mode". This interface is fully navigatible with a controller. Currently it's limited to the NoGUI frontend, but it will be available directly in the Qt frontend in the near future, with more features being added (e.g. game grid) as well.
- 2021/01/24: Runahead added - work around input lag in some games by running frames ahead of time and backtracking on input. DuckStation's implementation works with upscaling and the hardware renderers, but you still require a powerful computer for higher frame counts.
- 2021/01/24: Rewind added - you can now "smooth rewind" (but not for long), or "skip rewind" (for much long) while playing.
- 2021/01/10: Option to sync to host refresh rate added (enabled by default). This will give the smoothest animation possible with zero duped frames, at the cost of running the game <1% faster. Users with variable refresh rate (GSync/FreeSync) displays will want to disable the option.
- 2021/01/10: Audio resampling added when fast forwarding to fixed speeds. Instead of crackling audio, you'll now get pitch altered audio.
- 2021/01/03: Per game settings and game properties added to Android version.
- 2020/12/30: Box and Adaptive downsampling modes added. Adaptive downsampling will smooth 2D backgrounds but attempt to preserve 3D geometry via pixel similarity (only supported in D3D11/Vulkan). Box is a simple average filter which will downsample to native resolution.
- 2020/12/30: Hotkey binding added to Android version. You can now bind hotkeys such as fast forward, save state, etc to controller buttons. The ability to bind multi-button combinations will be added in the future.
- 2020/12/29: Controller mapping/binding added for Android version. By default mappings will be clear and you will have to set them, you can do this from `Settings -> Controllers -> Controller Mapping`. Profiles can be saved and loaded as well.
- 2020/12/29: Dark theme added for Android. By default it will follow your system theme (Android 10+), but can be overridden in settings.
- 2020/12/29: DirectInput/DInput controller interface added for Windows. You can use this if you are having difficulties with SDL. Vibration is not supported yet.
- 2020/12/25: Partial texture replacement support added. For now, this is only applicable to a small number of games which upload backgrounds to video RAM every frame. Dumping and replacement options are available in `Advanced Settings`.
- 2020/12/22: PGXP Depth Buffer enhancement added. This enhancement can eliminate "polygon fighting" in games, by giving the PS1 the depth buffer it never had. Compatibility is rather low at the moment, but for the games it does work in, it works very well. The depth buffer will be made available to postprocessing shaders in the future, enabling  effects such as SSAO.
- 2020/12/21: DuckStation now has two releases - Development and Preview. New features will appear in Preview first, and make their way to the Development release a few days later. To switch to preview, update to the latest development build (older builds will update to development), change the channel from `latest` to `preview` in general settings, and click `Check for Updates`.
- 2020/12/16: Integrated CPU debugger added in Qt frontend.
- 2020/12/13: Button layout for the touchscreen controller in the Android version can now be customized.
- 2020/12/10: Translation support added for Android version. Currently Brazillian Portuguese, Italian, and Dutch are available.
- 2020/11/27: Cover support added for game list in Android version. Procedure is the same as the desktop version, except you should place cover images in `<storage>/duckstation/covers` (see [Adding Game Covers](https://github.com/stenzek/duckstation/wiki/Adding-Game-Covers)).
- 2020/11/27: Disc database is shipped with desktop and Android versions courtesy of redump.org. This will provide titles for games on Android, where it was not possible previously.
- 2020/11/27: Compatibility databases added to libretro core - broken enhancements will be automatically disabled. You can turn this off by disabling "Apply Compatibility Settings" in the core options.
- 2020/11/27: SDL game controller database is included with desktop versions courtesy of https://github.com/gabomdq/SDL_GameControllerDB.
- 2020/11/21: OpenGL ES 2.0 host display support added. You cannot use the hardware renderer with GLES2, it still requires GLES3, but GLES2 GPUs can now use the software renderer.
- 2020/11/21: Threaded renderer for software renderer added. Can result in a significant speed boost depending on the game.
- 2020/11/21: AArch32/armv7 recompiler added. Android and Linux builds will follow after further testing, but for now you can build it yourself.
- 2020/11/18: Window size (resize window to Nx content resolution) added to Qt and SDL frontends.
- 2020/11/10: Widescreen hack now renders in the display aspect ratio instead of always 16:9.
- 2020/11/01: Exclusive fullscreen option added for Windows D3D11 users. Enjoy buttery smooth PAL games.
- 2020/10/31: Multisample antialiasing added as an enhancement.
- 2020/10/30: Option to use analog stick as d-pad for analog controller added.
- 2020/10/20: New cheat manager with memory scanning added. More features will be added over time.
- 2020/10/05: CD-ROM read speedup enhancement added.
- 2020/09/30: CPU overclocking is now supported. Use with caution as it will break games and increase system requirements. It can be set globally or per-game.
- 2020/09/25: Cheat support added for libretro core.
- 2020/09/23: Game covers added to Qt frontend (see [Adding Game Covers](https://github.com/stenzek/duckstation/wiki/Adding-Game-Covers)).
- 2020/09/19: Memory card importer/editor added to Qt frontend.
- 2020/09/13: Support for chaining post processing shaders added.
- 2020/09/12: Additional texture filtering options added.
- 2020/09/09: Basic cheat support added. Not all instructions/commands are supported yet.
- 2020/09/01: Many additional user settings available, including memory cards and enhancements. Now you can set these per-game.
- 2020/08/25: Automated builds for macOS now available.
- 2020/08/22: XInput controller backend added.
- 2020/08/20: Per-game setting overrides added. Mostly for compatibility, but some options are customizable.
- 2020/08/19: CPU PGXP mode added. It is very slow and incompatible with the recompiler, only use for games which need it.
- 2020/08/15: Playlist support/single memcard for multi-disc games in Qt frontend added.
- 2020/08/07: Automatic updater for standalone Windows builds.
- 2020/08/01: Initial PGXP (geometry/perspective correction) support.
- 2020/07/28: Qt frontend supports displaying interface in multiple languages.
- 2020/07/23: m3u multi-disc support for libretro core.
- 2020/07/22: Support multiple bindings for each controller button/axis.
- 2020/07/18: Widescreen hack enhancement added.
- 2020/07/04: Vulkan renderer now available in libretro core.
- 2020/07/02: Now available as a libretro core.
- 2020/07/01: Lightgun support with custom crosshairs.
- 2020/06/19: Vulkan hardware renderer added.