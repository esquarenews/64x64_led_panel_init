require "json"

module Speculum
  class ProcessManager
    def running?
      return false unless pid

      Process.kill(0, pid)
      true
    rescue Errno::ESRCH, Errno::EPERM
      false
    end

    def pid
      return unless Paths.pidfile.exist?

      Paths.pidfile.read.to_i
    end

    def status
      running? ? "Running" : "Paused"
    end

    def start(settings)
      raise "Speculum is already running" if running?

      FileUtils.mkdir_p(Paths.runtime_root)
      FileUtils.rm_f(Paths.state_file)
      FileUtils.rm_f(Paths.queue_file)
      command = PlayerCommand.new(settings).argv
      log = File.open(Paths.logfile, "a")
      log.sync = true
      child_pid = Process.spawn(*command, chdir: Paths.project_root.to_s, out: log, err: log, pgroup: true)
      Paths.pidfile.write(child_pid.to_s)
      child_pid
    ensure
      log&.close
    end

    def pause
      return unless running?

      Process.kill("TERM", -pid)
      wait_for_exit
    rescue Errno::ESRCH
      nil
    ensure
      Paths.pidfile.delete if Paths.pidfile.exist? && !running?
    end

    def restart(settings)
      pause if running?
      start(settings)
    end

    def recent_log(lines: 12)
      return [] unless Paths.logfile.exist?

      tail_lines(Paths.logfile, lines).map(&:strip)
    end

    def preview(library, settings)
      folder = settings["selected_folder"]
      if (state = player_state)
        current = state["current"]
        next_image = queued_image_name.presence || state["next"]
        return {
          current: current && library.image_record_for(folder, current),
          next: next_image && library.image_record_for(folder, next_image),
          timer: timer_state(state) || timer_state(displaying_state)
        }
      end

      display_state = displaying_state
      current = display_state&.dig("current") || current_image_name
      current_record = current && library.image_record_for(folder, current)
      current = nil unless current_record

      queued = queued_image_name
      queued_record = queued && library.image_record_for(folder, queued)
      queued = nil unless queued_record

      image_names = library.image_names(folder, limit: 250)
      current ||= image_names.first
      next_image = queued || next_image_name(image_names, current)
      {
        current: current_record || (current && library.image_record_for(folder, current)),
        next: queued_record || (next_image && library.image_record_for(folder, next_image)),
        timer: display_state && timer_state(display_state)
      }
    end

    private

    def timer_state(state)
      return unless state

      duration = state["dwell_seconds"].to_i
      started_at = state["updated_at"].to_s
      return if duration <= 0 || started_at.blank?

      {
        started_at: started_at,
        duration: duration
      }
    end

    def player_state
      return unless Paths.state_file.exist?

      state = JSON.parse(Paths.state_file.read)
      return unless state.is_a?(Hash)

      state
    rescue JSON::ParserError, Errno::ENOENT
      nil
    end

    def displaying_state
      return unless Paths.logfile.exist?

      recent_log(lines: 120).reverse_each do |line|
        if (match = line.match(/\ADisplaying (.+) for (\d+) seconds\.\.\.\z/))
          return {
            "current" => match[1],
            "dwell_seconds" => match[2].to_i,
            "updated_at" => Paths.logfile.mtime.utc.iso8601
          }
        end
      end
      nil
    rescue Errno::ENOENT
      nil
    end

    def wait_for_exit
      20.times do
        break unless running?
        sleep 0.1
      end
      Process.kill("KILL", -pid) if running?
    end

    def current_image_name
      recent_log(lines: 100).reverse_each do |line|
        if (match = line.match(/\ASending (.+)\.\.\.\z/))
          return match[1]
        end
      end
      nil
    end

    def next_image_name(image_names, current)
      return if image_names.empty?
      return image_names.first unless current

      index = image_names.index(current) || -1
      image_names[(index + 1) % image_names.length]
    end

    def queued_image_name
      return unless Paths.queue_file.exist?

      Paths.queue_file.read.strip
    end

    def tail_lines(path, lines)
      max_bytes = 128 * 1024
      File.open(path, "rb") do |file|
        size = file.size
        file.seek([size - max_bytes, 0].max)
        file.read.split("\n").last(lines) || []
      end
    rescue Errno::ENOENT
      []
    end
  end
end
