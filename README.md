# xnu_kernel_mediatek_common

The XNU kernel common to Open Mapple's MediaTek phones: Apple's xnu-8796.141.3, with the patch series of
ios-sweet and ios-viva as commits, each naming its origin. Built by `mka kernel` (macOS with Xcode). The SoC's
own parts come from `kernel/mediatek/<soc>`.

Checked out at `kernel/mediatek/common` by the [Open Mapple manifest](https://github.com/open-mapple/manifest), which says how to get, build and install it.

Licence: Apple Public Source License 2.0 (`APPLE_LICENSE`) for Apple's files and the changes to them; MIT for the files ios-sweet and ios-viva added. Details in `NOTICE`.
