# LandShark

A handheld SDR scanner running on the Waveshare [ESP32-P4-NANO](https://www.waveshare.com/esp32-p4-nano.htm?srsltid=AfmBOoqwx_UtnddP57XurmPjLDD6xyBxvlo3kfWMzl45RvUZGmMNA4tY) with an [RTL-SDR Blog V4](https://www.ebay.com/str/rtlsdrblog?_trksid=p4429486.m3561.l161211) plugged into its USB host port. Two apps right now: ADS-B aircraft tracking and P25 Phase 1 voice decode. Plan is to add more.

The whole thing is one firmware image. You hold a button or hit a key to switch between apps and the radio retunes on the fly. There's a TUI over the USB serial console (115200 8N1) so you can poke at it from a laptop, and an audio output through the onboard ES8311 codec for voice and confirmation beeps.

## What's working

ADS-B at 1090 MHz pulls Mode-S messages, decodes ICAO, callsign, altitude, position, velocity, vertical rate, and shows them in a live table. Reasonable range with a half-decent antenna, though indoors it's just whatever's directly overhead.

P25 Phase 1 voice on a known control channel will lock NAC/TG/SRC and play decoded IMBE+ voice through the codec at 8 kHz upsampled to 16. Works on real traffic — tested mostly against NH State Police on 154.785 MHz. The DSD decoder pipeline is grafted from upstream DSD with the ESP-IDF-specific bits rewritten and a shared sample ring between the IQ-read task and the frame-sync task.

The RTL-SDR V4 detection is in there with the right triplexer cable selection (HF→cable_2_in with 28.8 MHz upconversion, VHF→cable_1_in, UHF→air_in). Without that, VHF would have a 20 dB loss versus what it should get.

USB host runs at full bulk-IN rate (3.9 MB/s for ADS-B at 2 MSPS, 1.4 MB/s for P25 at 960 kSps). Double-buffered transfers, no FIFO overflow.

The TUI has four pages: a per-app MAIN page (different content for each app), a per-app SIGNAL page (Mode-S burst counter for ADS-B, decode-rate sparkline for P25), a shared LOG page, and a SETTINGS page with frequency/gain edit, favourites slots, and TTS voice tuning for the boot announcer.

## What's a work in progress

Sync isn't reliably hitting on real P25 signals at low input levels. With a stronger signal or AGC it should work, but the demod is sensitive to gain and the signal path matters a lot. I'm still figuring out the right gain/antenna combo for VHF reception in central NH.

The boot announcer uses SAM TTS (the Software Automatic Mouth) reading a preset phrase. It's there mostly because it was funny but it works.

There's also a stub for analog NFM monitoring (`apps/fd/`) intended for the Lakes Region Mutual Fire Aid F-1 frequency at 159.900, which I had basically working in an earlier flat-layout iteration but haven't ported into the current multi-app framework yet.

## What needs doing

- VHF analog monitor app needs to be properly ported into the current `apps/` layout. The DSP chain (CIC÷4 → box÷10 → atan2 → 7-tap FIR → 750 µs de-emph → ÷3) is all still in the source tree, just not wired into the current radio framework's app dispatcher.
- POCSAG decoder. The DSP front end would share most of the FM analog chain. Frame sync, BCH, address decode, message reassembly. Not starting until the analog NFM monitor is back working.
- Persist last-active app in NVS so it boots back into whichever app you were using.
- The on_sample dispatch path in the radio framework was originally meant to let any app share the bulk-read loop. P25 doesn't use it (it owns its own loop because of the different sample rate and DSP cadence), so the framework is doing double duty in a confusing way. Should clean that up.
- Stale unused-variable / unused-function warnings in librtlsdr.c, the tuner files, and various P25 sources are suppressed with `-Wno-error=*` rather than fixed. Mostly dead code paths preserved from upstream rtl-sdr / DSD.

## Future maybe-features

DMR voice would be the obvious next protocol after POCSAG since the DSP front end is similar to P25 and DSD already handles it. NXDN similarly.

Spectrum scanner mode that sweeps a band looking for signals would be useful, especially before pointing P25 at a frequency.

A small web UI served over the ESP32-P4's wireless capabilities, instead of the UART TUI, would be nicer to use day to day. The TUI is fine for development but it's tied to a USB cable.

Recording IQ to SD card for offline analysis. The V4 has the bandwidth, the P4 has the PSRAM, and the partition layout already includes space.

## Hardware notes

You need a Waveshare ESP32-P4-NANO specifically because of how the USB host pinout and PSRAM are wired. Other ESP32-P4 boards probably work but I haven't tried them. RTL-SDR V4 is the target dongle; older sticks would mostly work but you'd lose the triplexer routing.

Pin mapping for the audio: I²S MCLK=13 BCK=12 WS=10 DOUT=9 DIN=11, codec PA enable on GPIO 53, I²C SDA=7 SCL=8, USB VBUS enable on 46.

## Building

ESP-IDF v5.4.3, target `esp32p4`. From the project root:

```
idf.py set-target esp32p4
idf.py build
idf.py flash monitor
```

That's it. The `mbelib` (IMBE+ vocoder) is shipped as a local component under `components/` so there's no managed_components dance for that piece.

## Acknowledgements

DSD project for the P25 decoder logic. mbelib for the IMBE+ vocoder. rtl-sdr / rtlsdrblog for the librtlsdr base and the V4 triplexer logic. SAM TTS for the boot announcer. Espressif for the ESP-IDF and the ESP32-P4 HAL.
