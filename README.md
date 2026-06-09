# Six-Panel HUB75 Display

This project targets an ESP32 driving six 64×64 HUB75 panels arranged in a portrait zig-zag layout (two rows by three columns). The firmware streams RGB565 frames over BLE (or USB serial as a fallback) and renders a 2-second TV static transition between images.

## Firmware Highlights

- Configured for six chained panels using `CHAIN_TOP_RIGHT_DOWN_ZZ` mapping via `VirtualMatrixPanel_T`.
- Accepts RGB565 frames that are **128×192 pixels** (two bytes per pixel) over BLE/serial.
- Simple frame protocol:
  - Magic: `FRAM`
  - `uint16_t` width (little-endian)
  - `uint16_t` height
  - `uint8_t` format (`0` for RGB565)
  - `uint8_t` reserved (`0`)
  - `uint32_t` payload length (`width * height * 2`)
  - Binary payload containing packed RGB565 pixels
- Device emits status messages: `READY`, `OK`, `SHOW`, `TRANS`, and `DONE`.

> **Build note:** the CLI here does not ship with PlatformIO. Run `pip install platformio` (or use your IDE) before invoking `platformio run` / `pio run`.

## Ruby Image Player

The host utility `src/image_player.rb` keeps the display fed with images:

- Reads PNG and JPEG files from the `IMG/` directory.
- Uses ImageMagick via the `mini_magick` gem to crop/scale to 128×192, then converts to RGB565.
- Streams frames over BLE (default) or serial with minute-long dwell times and watches for device acknowledgements so that the two-second static effect can complete cleanly.

### Dependencies

```bash
gem install mini_magick serialport
pip install bleak  # required for BLE transport helper
brew install imagemagick  # or install from https://imagemagick.org/script/download.php
```

### Usage

```bash
cd /path/to/lrg-board-1
ruby src/image_player.rb --transport ble --ble-name LRGPanel --dwell 60
# Serial fallback example:
# ruby src/image_player.rb --transport serial --port auto --hard-reset
# Additional flags: --dir, --poll, --blur 0.6, --sharpen 0.8, --dither
```

Drop any 128×192 (or larger) PNG/JPG images into `IMG/`. The script rescales to fit, optionally applies blur/sharpen/dithering before RGB565 quantisation, waits for the firmware to display them for 60 seconds, and then advances to the next image while the board renders the TV static transition over BLE or serial.

## Speculum Web Controller

`speculum/` is a Rails control app for running the image player from a home server without SSH. It provides login, run/pause/reset controls, image upload/delete, folder rename/reorder, current/next previews, and UI for the player options.

```bash
cd speculum
bundle install
SPECULUM_USERNAME=admin SPECULUM_PASSWORD='change-me' bundle exec rails server -b 0.0.0.0 -p 3000
```

By default the app launches `../src/image_player.rb` with serial transport, `--port auto`, and `--hard-reset`. Set `SPECULUM_RUBY=/path/to/ruby` if the server needs a specific Ruby executable for the sender.

### Production deploy

On the production server, run this from the repository checkout:

```bash
./scripts/deploy_speculum.sh
```

The script pulls the current branch, installs the Rails bundle, precompiles assets, creates `/etc/speculum/speculum.env` on first run, installs/enables a `speculum` systemd service, and restarts it. Useful overrides:

```bash
PORT=3001 SERVICE_NAME=speculum ./scripts/deploy_speculum.sh
RUN_TESTS=1 ./scripts/deploy_speculum.sh
RESET_CREDENTIALS=1 SPECULUM_USERNAME=admin SPECULUM_PASSWORD='new-password' ./scripts/deploy_speculum.sh
```

If `/etc/speculum/speculum.env` already exists, the deploy keeps it unchanged so production credentials are not overwritten.
# 64x64_led_panel_init
