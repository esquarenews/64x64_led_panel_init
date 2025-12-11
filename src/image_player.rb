# frozen_string_literal: true

require 'optparse'
require 'fileutils'
require 'json'
require 'open3'
require 'base64'
require 'tempfile'

begin
  require 'serialport'
rescue LoadError
  SerialPort = nil
end

begin
  require 'mini_magick'
rescue LoadError
  warn 'The mini_magick gem is required. Install it with `gem install mini_magick`.'
  exit 1
end

DISPLAY_WIDTH = 192
DISPLAY_HEIGHT = 256
FRAME_MAGIC = 'FRAM'.b
FRAME_FORMAT = 0
DEFAULT_DWELL_SECONDS = 60
DEFAULT_POLL_SECONDS = 5
DEFAULT_TRANSPORT = 'serial'
DEFAULT_BLE_NAME = 'LRGPanel'
BLE_RECONNECT_DELAY = 5
MAX_RETRY_ATTEMPTS = 5
SUPPORTED_EXTENSIONS = %w[.png .jpg .jpeg].freeze
COLOR_GAMMA = 0.9
CHANNEL_GAIN = {
  r: 0.75,
  g: 1.30,
  b: 1.30
}.freeze

COLOR_GRID_SWATCHES = [
  [255, 0, 0],     # Red
  [0, 255, 0],     # Green
  [0, 0, 255],     # Blue
  [255, 255, 0],   # Yellow
  [0, 255, 255],   # Cyan
  [255, 0, 255],   # Magenta
  [255, 255, 255], # White
  [128, 128, 128], # Gray
  [0, 0, 0],       # Black
  [255, 128, 0],   # Orange
  [128, 0, 255],   # Purple
  [0, 128, 255]    # Light Blue
].freeze

PYTHON_BLE_BRIDGE = <<~'PYCODE'
import asyncio
import base64
import json
import sys

NUS_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
NUS_RX_CHAR_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
NUS_TX_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

try:
    from bleak import BleakClient, BleakScanner
except Exception as exc:
    print(json.dumps({"status": "error", "error": f"bleak not installed: {exc}"}), flush=True)
    sys.exit(1)

state = {
    "client": None,
    "target": None,
    "name": None,
    "address": None,
    "rx_char": None,
    "tx_char": None,
    "notifications": [],
    "notify_started": False,
}


def notification_handler(_, data):
    if data:
        state["notifications"].append(bytes(data))


async def ensure_services(client):
    services = getattr(client, "services", None)
    if services is not None:
        return services

    getter = getattr(client, "get_services", None)
    if getter is not None:
        services = await getter()
        if services is not None:
            return services

    backend = getattr(client, "_backend", None)
    if backend is not None and hasattr(backend, "_get_services"):
        services = await backend._get_services()
        if services is not None:
            return services

    services = getattr(client, "services", None)
    if services is None:
        raise RuntimeError("Service discovery failed")
    return services


async def discover_target():
    if state["target"]:
        return state["target"]
    name = state["name"]
    address = state["address"]
    if address:
        state["target"] = address
        return address
    try:
        if hasattr(BleakScanner, "find_device_by_filter"):
            def _flt(device, adv_data):
                if name and (device.name == name or (device.name or "").startswith(name)):
                    return True
                uuids = []
                try:
                    uuids = (adv_data and adv_data.service_uuids) or []
                except Exception:
                    uuids = []
                for uuid in uuids:
                    if str(uuid).lower() == NUS_SERVICE_UUID.lower():
                        return True
                return False

            device = await BleakScanner.find_device_by_filter(_flt, timeout=8.0)
            if device is not None:
                state["target"] = device.address
                return state["target"]
    except Exception:
        pass

    devices = await BleakScanner.discover(timeout=10.0)
    for dev in devices:
        if name and (dev.name == name or (dev.name or "").startswith(name)):
            state["target"] = dev.address
            return state["target"]
    for dev in devices:
        uuids = []
        try:
            uuids = dev.metadata.get("uuids") or []
        except Exception:
            uuids = []
        for uuid in uuids:
            if (uuid or "").lower() == NUS_SERVICE_UUID.lower():
                state["target"] = dev.address
                return state["target"]
    raise RuntimeError(f"BLE target not found (name={name!r} address={address!r})")


async def ensure_connected():
    client = state["client"]
    if client is not None and client.is_connected:
        if state.get("rx_char") is None or state.get("tx_char") is None:
            services = await ensure_services(client)
            if state.get("rx_char") is None:
                rx = services.get_characteristic(NUS_RX_CHAR_UUID)
                if rx:
                    state["rx_char"] = rx
            if state.get("tx_char") is None:
                tx = services.get_characteristic(NUS_TX_CHAR_UUID)
                if tx and not state["notify_started"]:
                    await client.start_notify(tx, notification_handler)
                    state["tx_char"] = tx
                    state["notify_started"] = True
        return client

    target = await discover_target()

    if client is not None:
        try:
            await client.disconnect()
        except Exception:
            pass

    client = BleakClient(target, timeout=12.0)
    await client.connect()
    if not client.is_connected:
        raise RuntimeError("BLE connect failed")

    services = await ensure_services(client)
    rx = services.get_characteristic(NUS_RX_CHAR_UUID)
    tx = services.get_characteristic(NUS_TX_CHAR_UUID)
    if rx is None or tx is None:
        raise RuntimeError("Required characteristics not found")

    state["client"] = client
    state["rx_char"] = rx
    state["tx_char"] = tx
    if not state["notify_started"]:
        await client.start_notify(tx, notification_handler)
        state["notify_started"] = True

    return client


async def write_payload(payload_b64):
    client = await ensure_connected()
    data = base64.b64decode(payload_b64)
    if not data:
        return
    rx_char = state.get("rx_char")
    if rx_char is None:
        services = await ensure_services(client)
        rx_char = services.get_characteristic(NUS_RX_CHAR_UUID)
        if rx_char is None:
            raise RuntimeError("RX characteristic unavailable")
        state["rx_char"] = rx_char

    mtu = None
    for attr in ("mtu_size", "mtu"):
        mtu = getattr(client, attr, None)
        if mtu:
            break
    if not mtu:
        mtu = 247
    chunk = max(20, int(mtu) - 3)

    for offset in range(0, len(data), chunk):
        chunk_data = data[offset:offset + chunk]
        await client.write_gatt_char(rx_char, chunk_data, response=True)


async def read_notifications(timeout):
    await ensure_connected()
    timeout = max(0.0, float(timeout or 0.0))
    if timeout == 0:
        if state["notifications"]:
            data = b"".join(state["notifications"])
            state["notifications"].clear()
            return data.decode("utf-8", "ignore")
        return ""

    loop = asyncio.get_event_loop()
    deadline = loop.time() + timeout

    while True:
        if state["notifications"]:
            data = b"".join(state["notifications"])
            state["notifications"].clear()
            return data.decode("utf-8", "ignore")
        if loop.time() >= deadline:
            return ""
        await asyncio.sleep(0.05)


async def close_client():
    client = state.get("client")
    if client is not None:
        try:
            if state.get("notify_started") and state.get("tx_char") is not None:
                try:
                    await client.stop_notify(state["tx_char"])
                except Exception:
                    pass
        finally:
            try:
                await client.disconnect()
            except Exception:
                pass
    state["client"] = None
    state["rx_char"] = None
    state["tx_char"] = None
    state["notify_started"] = False
    state["notifications"].clear()


async def handle_command(cmd):
    command = cmd.get("type")
    if command == "config":
        state["name"] = cmd.get("name")
        state["address"] = cmd.get("address")
        if state["address"]:
            state["target"] = state["address"]
        return "configured"
    if command == "connect":
        await ensure_connected()
        return "connected"
    if command == "write":
        payload = cmd.get("payload", "")
        await write_payload(payload)
        return "written"
    if command == "read":
        timeout = cmd.get("timeout", 0)
        data = await read_notifications(timeout)
        return data
    if command == "close":
        await close_client()
        return "closed"
    raise RuntimeError(f"Unknown command type: {command}")


loop = asyncio.new_event_loop()
asyncio.set_event_loop(loop)

for line in sys.stdin:
    line = line.strip()
    if not line:
        continue
    try:
        cmd = json.loads(line)
    except Exception as exc:
        print(json.dumps({"status": "error", "error": f"invalid json: {exc}"}), flush=True)
        continue
    try:
        result = loop.run_until_complete(handle_command(cmd))
        print(json.dumps({"status": "ok", "result": result}), flush=True)
        if cmd.get("type") == "close":
            break
    except Exception as exc:
        print(json.dumps({"status": "error", "error": str(exc)}), flush=True)
        if cmd.get("type") == "close":
            break

loop.run_until_complete(close_client())
PYCODE

class BleBridge
  def initialize_helper(name, address)
    @name = name
    @address = address
    @stdin, @stdout, @wait_thr = Open3.popen2('python3', '-u', '-c', PYTHON_BLE_BRIDGE)
    send_command(type: 'config', name: name, address: address)
    read_ok
  rescue Errno::ENOENT
    cleanup
    raise 'python3 not found; install it to enable BLE transport'
  end
  private :initialize_helper

  def initialize(name:, address: nil)
    initialize_helper(name, address)
  rescue StandardError => e
    cleanup
    raise e
  end

  def connect
    send_command(type: 'connect')
    read_ok
  end

  def write_bytes(bytes)
    payload = Base64.strict_encode64(bytes)
    send_command(type: 'write', payload: payload)
    read_ok
  rescue StandardError => e
    raise e
  end

  def read_notifications(timeout)
    send_command(type: 'read', timeout: timeout)
    read_ok || ''
  end

  def close
    send_command(type: 'close')
    read_ok
  rescue StandardError
    nil
  ensure
    cleanup
  end

  private

  def ensure_alive!
    raise 'BLE helper exited unexpectedly' unless @wait_thr&.alive?
  end

  def send_command(payload)
    ensure_alive!
    @stdin.puts(JSON.generate(payload))
    @stdin.flush
  end

  def read_response
    ensure_alive!
    line = @stdout.gets
    raise 'BLE helper terminated unexpectedly' unless line
    JSON.parse(line)
  rescue JSON::ParserError => e
    raise "Invalid response from BLE helper: #{e.message}"
  end

  def read_ok
    loop do
      resp = read_response
      status = resp['status']
      return resp['result'] if status == 'ok'
      raise(resp['error'] || 'BLE helper returned error') if status == 'error'
      # Ignore other statuses (e.g., debug logs)
    end
  end

  def cleanup
    begin
      @stdin&.close unless @stdin&.closed?
    rescue StandardError
      nil
    end
    begin
      @stdout&.close unless @stdout&.closed?
    rescue StandardError
      nil
    end
    begin
      @wait_thr&.value
    rescue StandardError
      nil
    end
  end
end

class BleTransport
  def initialize(name:, address: nil)
    @bridge = BleBridge.new(name: name, address: address)
    @buffer = +' '
    @buffer.clear
    connect
  end

  def connect
    @bridge.connect
  end

  def write(data)
    bytes = data.is_a?(String) ? data : data.to_s
    @bridge.write_bytes(bytes.b)
  end

  def read_line(timeout:)
    deadline = Process.clock_gettime(Process::CLOCK_MONOTONIC) + timeout
    loop do
      if (idx = @buffer.index("\n"))
        line = @buffer.slice!(0..idx)
        return line.delete("\r\n")
      end

      remaining = deadline - Process.clock_gettime(Process::CLOCK_MONOTONIC)
      return nil if remaining <= 0

      chunk = @bridge.read_notifications(remaining)
      next unless chunk && !chunk.empty?

      @buffer << chunk
    end
  end

  def close
    @bridge.close
  end
end

class SerialTransport
  def initialize(port:, baud: 115_200)
    raise 'The serialport gem is required. Install it with `gem install serialport`.' unless defined?(SerialPort) && SerialPort

    @port = port
    @port_pattern = derive_port_pattern(port)
    @baud = baud
    @serial = nil
    @buffer = +' '
    @buffer.clear
    open_serial
  end

  def write(data)
    ensure_serial
    payload = data.is_a?(String) ? data.b : data.to_s.b
    offset = 0
    while offset < payload.bytesize
      chunk = payload.byteslice(offset, 128)
      written = @serial.write(chunk)
      raise IOError, 'Write returned nil' unless written
      offset += written
      @serial.flush
      sleep 0.003
    end
  rescue Errno::ENXIO, Errno::EIO, IOError
    reopen_with_retry
    retry
  end

  def read_line(timeout:)
    ensure_serial
    deadline = Process.clock_gettime(Process::CLOCK_MONOTONIC) + timeout
    loop do
      if (idx = @buffer.index("\n"))
        line = @buffer.slice!(0..idx)
        return line.delete("\r\n")
      end

      remaining = deadline - Process.clock_gettime(Process::CLOCK_MONOTONIC)
      return nil if remaining <= 0

      ready = IO.select([@serial], nil, nil, remaining)
      next unless ready

      chunk = @serial.read(1)
      next unless chunk

      @buffer << chunk
    rescue Errno::ENXIO, Errno::EIO, IOError
      reopen_with_retry
      next
    end
  end

  def close
    @serial&.close unless @serial&.closed?
    @serial = nil
  end

  private

  def open_serial
    @serial = SerialPort.new(@port, @baud)
    begin
      @serial.read_timeout = 1000 if @serial.respond_to?(:read_timeout=)
      # Keep reset/boot lines deasserted; low values can reboot the ESP32.
      @serial.dtr = 1 if @serial.respond_to?(:dtr=)
      @serial.rts = 1 if @serial.respond_to?(:rts=)
      @serial.flow_control = SerialPort::NONE if @serial.respond_to?(:flow_control=)
    rescue StandardError
      nil
    end
    sleep 0.1
  end

  def ensure_serial
    return if @serial && !@serial.closed?

    open_serial
  end

  def reopen_with_retry(max_attempts: nil, delay: 0.25)
    attempts = 0
    loop do
      attempts += 1
      begin
        close
        open_serial
        return
      rescue StandardError => e
        if max_attempts && attempts >= max_attempts
          raise "Serial device unavailable after #{attempts} attempts: #{e.class}: #{e.message}"
        end
        warn "Serial reconnect attempt #{attempts} failed: #{e.class}: #{e.message} (retrying...)" if attempts == 1 || (attempts % 10).zero?
        if e.is_a?(Errno::ENOENT)
          if (replacement = find_replacement_port)
            warn "Serial port changed, switching to #{replacement}"
            @port = replacement
          end
        end
        sleep delay
      end
    end
  end

  def derive_port_pattern(port)
    base = File.basename(port)
    if (m = base.match(/([a-zA-Z_.]+)\d+/))
      m[1]
    else
      base
    end
  end

  def find_replacement_port
    return nil unless @port_pattern
    Dir['/dev/cu.*'].find { |p| File.basename(p).start_with?(@port_pattern) }
  end
end

def clamp255(value)
  [[value.round, 0].max, 255].min
end

def apply_color_correction(value, gain)
  normalized = value.to_f / 255.0
  corrected = normalized**COLOR_GAMMA
  corrected *= gain
  clamp255(corrected * 255.0)
end

def build_color_grid_payload(width, height, apply_correction: true)
  cols = 4
  rows = (COLOR_GRID_SWATCHES.length / cols.to_f).ceil
  block_w = [width / cols, 1].max
  block_h = [height / rows, 1].max

  payload = String.new(capacity: width * height * 2, encoding: Encoding::BINARY)

  height.times do |y|
    swatch_row = [[y / block_h, rows - 1].min, 0].max
    width.times do |x|
      swatch_col = [[x / block_w, cols - 1].min, 0].max
      swatch_idx = swatch_row * cols + swatch_col
      swatch = COLOR_GRID_SWATCHES[swatch_idx % COLOR_GRID_SWATCHES.length]

      r, g, b = swatch
      if apply_correction
        r = apply_color_correction(r, CHANNEL_GAIN[:r])
        g = apply_color_correction(g, CHANNEL_GAIN[:g])
        b = apply_color_correction(b, CHANNEL_GAIN[:b])
      end

      r5 = ((r * 31) + 127) / 255
      g6 = ((g * 63) + 127) / 255
      b5 = ((b * 31) + 127) / 255
      payload << [ (r5 << 11) | (g6 << 5) | b5 ].pack('S<')
    end
  end

  payload
end

def build_static_payload(width, height, apply_correction: true)
  rng = Random.new
  payload = String.new(capacity: width * height * 2, encoding: Encoding::BINARY)

  (width * height).times do
    r = rng.rand(0..255)
    g = rng.rand(0..255)
    b = rng.rand(0..255)

    if apply_correction
      r = apply_color_correction(r, CHANNEL_GAIN[:r])
      g = apply_color_correction(g, CHANNEL_GAIN[:g])
      b = apply_color_correction(b, CHANNEL_GAIN[:b])
    end

    r5 = ((r * 31) + 127) / 255
    g6 = ((g * 63) + 127) / 255
    b5 = ((b * 31) + 127) / 255
    payload << [ (r5 << 11) | (g6 << 5) | b5 ].pack('S<')
  end

  payload
end

def countdown_message(today = Date.today)
  target = Date.new(today.year, 12, 25)
  target = Date.new(today.year + 1, 12, 25) if today > target
  return 'Merry Christmas' if today == target
  days = (target - today).to_i
  "#{days} days until Christmas"
end

def draw_overlay(image, text)
  return if text.nil? || text.empty?
  safe = text.gsub("'", "\\\\'")
  image.combine_options do |c|
    c.gravity 'SouthEast'
    c.fill 'white'
    c.stroke 'none'
    c.strokewidth 0
    c.font 'Courier'
    c.pointsize 12
    c.draw "text 4,4 '#{safe}'"
  end
end

def prepare_payload(path, width, height, blur: nil, sharpen: nil, dither: false, color_correct: true)
  image = MiniMagick::Image.open(path)

  image.resize "#{width}x#{height}!"
  image.colorspace 'RGB'
  image.blur "0x#{blur}" if blur
  image.sharpen "0x#{sharpen}" if sharpen
  if block_given?
    yield image
  end
  if dither
    image.dither 'FloydSteinberg'
    image.colors 65536
  end

  pixels = image.get_pixels
  if pixels.length != height || pixels.any? { |row| row.length != width }
    raise "Image resize failed: got #{pixels.length}x#{pixels.first&.length}"
  end

  payload = String.new(capacity: width * height * 2, encoding: Encoding::BINARY)
  pixels.each do |row|
    row.each do |pixel|
      r, g, b = pixel[0, 3] || [0, 0, 0]
      if color_correct
        r = apply_color_correction(r, CHANNEL_GAIN[:r])
        g = apply_color_correction(g, CHANNEL_GAIN[:g])
        b = apply_color_correction(b, CHANNEL_GAIN[:b])
      end
      r5 = ((r * 31) + 127) / 255
      g6 = ((g * 63) + 127) / 255
      b5 = ((b * 31) + 127) / 255
      payload << [ (r5 << 11) | (g6 << 5) | b5 ].pack('S<')
    end
  end

  payload
rescue MiniMagick::Error => e
  warn "Failed to process #{path}: #{e.message}"
  nil
end

def send_frame(transport, width, height, payload)
  raise ArgumentError, 'Payload size mismatch' if payload.bytesize != width * height * 2

  header = [
    FRAME_MAGIC,
    width,
    height,
    FRAME_FORMAT,
    0,
    payload.bytesize
  ].pack('a4S<S<CCL<')

  transport.write(header + payload)
end

def discover_images(dir)
  Dir.children(dir)
     .select { |name| SUPPORTED_EXTENSIONS.include?(File.extname(name).downcase) }
     .sort
     .map { |name| File.join(dir, name) }
rescue Errno::ENOENT
  []
end

def wait_for(transport, expected, timeout:, optional: false)
  expectations = Array(expected).map(&:to_s)
  loop do
    line = transport.read_line(timeout: timeout)
    return nil if line.nil? && optional
    raise "Timeout waiting for #{expectations.join('/')}" unless line

    puts "[device] #{line}"
    return line if expectations.include?(line)
  end
end

options = {
  transport: DEFAULT_TRANSPORT,
  port: nil,
  baud: 115_200,
  ble_name: DEFAULT_BLE_NAME,
  ble_address: nil,
  image_dir: File.expand_path('../IMG', __dir__),
  dwell: DEFAULT_DWELL_SECONDS,
  poll: DEFAULT_POLL_SECONDS,
  blur: nil,
  sharpen: nil,
  dither: false,
  manual: false,
  color_correct: true,
  countdown: true,
  test_pattern: false,
  static_test: false
}

OptionParser.new do |opts|
  opts.banner = 'Usage: ruby src/image_player.rb [options]'
  opts.on('-t', '--transport MODE', 'Transport: serial or ble (default: ble)') do |value|
    options[:transport] = value.downcase
  end
  opts.on('-p', '--port PATH', 'Serial port device (for serial transport)') { |value| options[:port] = value }
  opts.on('-b', '--baud RATE', Integer, "Serial baud rate (default: #{options[:baud]})") { |value| options[:baud] = value }
  opts.on('--ble-name NAME', 'BLE device name (default: LRGPanel)') { |value| options[:ble_name] = value }
  opts.on('--ble-address ADDR', 'BLE address override') { |value| options[:ble_address] = value }
  opts.on('-d', '--dir PATH', 'Directory containing source images (default: ../IMG)') do |value|
    options[:image_dir] = File.expand_path(value)
  end
  opts.on('--dwell SECONDS', Integer, "Seconds to display each image (default: #{options[:dwell]})") do |value|
    options[:dwell] = value if value.positive?
  end
  opts.on('--poll SECONDS', Integer, "Seconds between scans when no images are found (default: #{options[:poll]})") do |value|
    options[:poll] = value if value.positive?
  end
  opts.on('--blur SIGMA', Float, 'Apply Gaussian blur before scaling (sigma value)') do |value|
    options[:blur] = value if value.positive?
  end
  opts.on('--sharpen SIGMA', Float, 'Apply sharpen after scaling (sigma value)') do |value|
    options[:sharpen] = value if value.positive?
  end
  opts.on('--dither', 'Enable Floyd-Steinberg dithering before RGB565 quantization') do
    options[:dither] = true
  end
  opts.on('--manual', 'Advance images manually (press Enter) instead of waiting dwell time') do
    options[:manual] = true
  end
  opts.on('--no-color-correct', 'Disable color correction / gamma adjustments') do
    options[:color_correct] = false
  end
  opts.on('--no-countdown', 'Do not overlay days-until-Christmas text') do
    options[:countdown] = false
  end
  opts.on('--test-pattern', 'Send built-in colour calibration pattern') do
    options[:test_pattern] = true
  end
  opts.on('--static-test', 'Stream a looping static noise test pattern') do
    options[:static_test] = true
  end
  opts.on('-h', '--help', 'Show this help message') do
    puts opts
    exit
  end
end.parse!

FileUtils.mkdir_p(options[:image_dir])

# transport handle is initialised after helper definitions
transport = nil

def build_transport(options)
  case options[:transport]
  when 'serial'
    raise 'Provide --port when using serial transport.' if options[:port].nil? || options[:port].empty?
    SerialTransport.new(port: options[:port], baud: options[:baud])
  when 'ble'
    BleTransport.new(name: options[:ble_name], address: options[:ble_address])
  else
    raise "Unsupported transport: #{options[:transport]}"
  end
end

def ensure_transport(transport, options)
  return transport if transport

  loop do
    begin
      return build_transport(options)
    rescue StandardError => e
      warn "Transport init failed: #{e.message}. Retrying in #{BLE_RECONNECT_DELAY}s..."
      sleep BLE_RECONNECT_DELAY
    end
  end
end

def wait_for_ready(transport, options)
  attempts = 0
  puts 'Waiting for controller...'
  loop do
    line = transport.read_line(timeout: 1)
    unless line
      warn "No READY response, retrying transport init in #{BLE_RECONNECT_DELAY}s..."
      sleep BLE_RECONNECT_DELAY
      begin
        transport&.close
      rescue StandardError
        nil
      end
      transport = ensure_transport(nil, options)
      attempts += 1
      raise 'Controller did not respond to READY' if attempts >= MAX_RETRY_ATTEMPTS
      next
    end
    puts "[device] #{line}"
    break if line == 'READY'
  end
  transport
end

transport = ensure_transport(nil, options)
begin
  transport = wait_for_ready(transport, options)
rescue StandardError => e
  warn "Failed to establish controller READY state: #{e.message}"
  exit 1
end

at_exit do
  begin
    transport&.close
  rescue StandardError
    nil
  end
end

frames_sent = 0

if options[:test_pattern]
  payload = build_color_grid_payload(
    DISPLAY_WIDTH,
    DISPLAY_HEIGHT,
    apply_correction: options[:color_correct]
  )
  puts 'Sending built-in colour grid...'
  send_frame(transport, DISPLAY_WIDTH, DISPLAY_HEIGHT, payload)
  wait_for(transport, 'OK', timeout: 20)
  wait_for(transport, %w[SHOW DONE], timeout: 20)
  puts 'Test pattern sent. Exiting.'
  exit
end

if options[:static_test]
  puts 'Streaming static noise test pattern (Ctrl+C to stop)...'
  loop do
    begin
      payload = build_static_payload(
        DISPLAY_WIDTH,
        DISPLAY_HEIGHT,
        apply_correction: options[:color_correct]
      )
      send_frame(transport, DISPLAY_WIDTH, DISPLAY_HEIGHT, payload)
      wait_for(transport, 'OK', timeout: 20)
      acknowledgement =
        if frames_sent.zero?
          wait_for(transport, %w[SHOW DONE], timeout: 20)
        else
          wait_for(transport, 'DONE', timeout: 20)
        end
      frames_sent += 1
      puts "Static frame #{frames_sent} acknowledged with #{acknowledgement}. Waiting for transition..."
      sleep 2.5
    rescue StandardError => e
      warn "Transport error: #{e.message}. Attempting reconnect..."
      transport = ensure_transport(nil, options)
      begin
        transport = wait_for_ready(transport, options)
      rescue StandardError => inner_e
        warn "Reconnect failed: #{inner_e.message}. Retrying in #{BLE_RECONNECT_DELAY}s..."
        sleep BLE_RECONNECT_DELAY
        retry
      end
    end
  end
end

loop do
  image_paths = discover_images(options[:image_dir])

  if image_paths.empty?
    warn "No images found in #{options[:image_dir]}. Rechecking in #{options[:poll]}s..."
    sleep options[:poll]
    next
  end

  image_paths.each do |image_path|
    begin
      overlay_text = options[:countdown] ? countdown_message : nil
      payload = prepare_payload(
        image_path,
        DISPLAY_WIDTH,
        DISPLAY_HEIGHT,
        blur: options[:blur],
        sharpen: options[:sharpen],
        dither: options[:dither],
        color_correct: options[:color_correct]
      ) do |img|
        draw_overlay(img, overlay_text) if overlay_text
      end
      next unless payload

      puts "Sending #{File.basename(image_path)}..."
      send_frame(transport, DISPLAY_WIDTH, DISPLAY_HEIGHT, payload)
      wait_for(transport, 'OK', timeout: 20)

      acknowledgement =
        if frames_sent.zero?
          wait_for(transport, %w[SHOW DONE], timeout: 20)
        else
          wait_for(transport, 'DONE', timeout: 20)
        end

      frames_sent += 1

      if options[:manual]
        puts "Received #{acknowledgement}. Press Enter to display the next image (Ctrl+C to quit)..."
        $stdin.gets
      else
        puts "Displaying #{File.basename(image_path)} for #{options[:dwell]} seconds..."
        sleep options[:dwell]
      end
    rescue StandardError => e
      warn "Transport error: #{e.message}. Attempting reconnect..."
      transport = ensure_transport(nil, options)
      begin
        transport = wait_for_ready(transport, options)
      rescue StandardError => inner_e
        warn "Reconnect failed: #{inner_e.message}. Retrying in #{BLE_RECONNECT_DELAY}s..."
        sleep BLE_RECONNECT_DELAY
        retry
      end
      retry
    end
  end
end
