# NXP Wi-Fi driver and OS abstraction

The driver for the NXP IW610 inside the Neuro N6's MAYA-W476 module, and the
operating-system layer it is written against, from NXP's MCUXpresso SDK.
Nothing here is edited in this repository.

| folder | what it is | terms |
|---|---|---|
| `wifi_nxp/` | NXP's `wifi_nxp` middleware from the MCUXpresso SDK, driver version `v1.3.r49.p2` (`incl/wlcmgr/wlan.h`): the connection manager, the 802.11 driver, the firmware download, the DHCP server, the FreeRTOS and lwIP ports. `LICENSE` is the folder's BSD-3-Clause licence. | per file, as their headers state: BSD-3-Clause for almost all; Apache-2.0 for `incl/port/net/wm_net.h` (with BSD-3-Clause) and `nw_utils/wifi_ping.c`; MIT for `port/osa/osa_freertos.c`, `incl/port/osa/slist.h` and `stack_simple.h`; the BSD licence of their authors (hostapd's Jouni Malinen, Cozybit) for most `wmcrypto/aescrypto` headers, whose `includes.h` and `type_def.h` carry NXP's copyright alone, under the folder's `LICENSE`; lwIP's BSD-3-Clause for `port/net/wifi_netif.c` |
| `osa/` | NXP's OS abstraction for FreeRTOS (`fsl_os_abstraction*`) and its list component, with `stubs/`: minimal stand-ins for the SDK headers the driver includes but this board has no use for | BSD-3-Clause, per file |

`wifi_nxp/LA_OPT_NXP_Software_License.txt` is NXP's proprietary licence. In
NXP's SDK it governs the proprietary parts, such as the radio firmware, which
this repository carries separately under
`Middlewares/NeuroN6/MayaW476/firmware`. The file headers above are what
they say of themselves.

`Examples/Camera_Nia416Stream_WiFi_FreeRTOS` compiles every file here. The driver's SDIO host layer
for NXP's own processors, `mlan_sdio.c`, is not carried: only its interface
is, in `sdio_nxp_abs/incl/mlan_sdio_api.h`, and
`Middlewares/NeuroN6/MayaW476/sdio_port` implements that for the STM32N6.
The switches and flags are in that example's `wifi_build.json`.
