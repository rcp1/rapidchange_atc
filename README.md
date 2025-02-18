# rapidchange_atc

[RapidChange](https://rapidchangeatc.com/) automatic tool change plugin for [grblHal](https://www.grbl.org/what-is-grblhal) based on the [RapidChange FluidNC version](https://github.com/greilick-industries/FluidNC-RapidChangeATC).

This plugin is not associated with Greilick Industries LLC.

> :warning: **Usage at your own risk, was only roughly tested on my machine**

## Todo

- [x] Tool change working
- [x] Use atc_init instead of my_plugin_init
- [ ] Setting variation tests
- [x] Error handling
- [x] Tool setter
- [ ] Ensuring that tool is not forgotten on errors / resets
- [x] Tool recognition
- [x] Dust cover
- [ ] Allow other orientations / axis of magazine than Z axis to load / unload

## Usage

Add this repository as submodule to your grblHal driver checkout, define `ATC_ENABLE` in your my_machine.h and re-compile.

So your my_machine.h needs to contain:

```c
#define ATC_ENABLE                1
```
