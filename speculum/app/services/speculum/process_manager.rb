require "json"

module Speculum
  class ProcessManager
    def running?
      current_pid = pid
      return false unless current_pid

      Process.kill(0, current_pid)
      if zombie?(current_pid)
        reap(current_pid)
        Paths.pidfile.delete if Paths.pidfile.exist?
        return false
      end
      true
    rescue Errno::ESRCH, Errno::EPERM
      Paths.pidfile.delete if Paths.pidfile.exist?
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
      if running?
        restart(settings)
        return :restarted
      end

      spawn_player(settings)
    end

    def pause
      current_pid = pid
      return clear_runtime_state unless current_pid && running?

      stop_pid(current_pid, force_clear: true)
    rescue Errno::ESRCH
      nil
    ensure
      clear_runtime_state
    end

    def restart(settings)
      current_pid = pid
      stop_pid(current_pid, force_clear: true) if current_pid
      clear_runtime_state
      spawn_player(settings)
    end

    def unhealthy?(max_startup_age: 90)
      return false unless running?
      state = player_state || displaying_state
      return !fresh_state?(state) if state

      if Paths.pidfile.exist?
        Time.now - Paths.pidfile.mtime > max_startup_age
      else
        true
      end
    end

    def recent_log(lines: 12)
      return [] unless Paths.logfile.exist?

      tail_lines(Paths.logfile, lines).map { |line| sanitize_log_line(line) }
    end

    def preview(library, settings)
      folder = settings["selected_folder"]
      if (state = player_state)
        if fresh_state?(state)
          current = state["current"]
          next_image = queued_image_name.presence || state["next"]
          return {
            current: current && library.image_record_for(folder, current),
            next: next_image && library.image_record_for(folder, next_image),
            timer: timer_state(state) || timer_state(displaying_state)
          }
        end
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

    def spawn_player(settings)
      FileUtils.mkdir_p(Paths.runtime_root)
      clear_runtime_state
      command = PlayerCommand.new(settings).argv
      log = File.open(Paths.logfile, "a")
      log.sync = true
      child_pid = Process.spawn(*command, chdir: Paths.project_root.to_s, out: log, err: log, pgroup: true)
      Paths.pidfile.write(child_pid.to_s)
      child_pid
    ensure
      log&.close
    end

    def clear_runtime_state
      FileUtils.rm_f(Paths.pidfile)
      FileUtils.rm_f(Paths.state_file)
      FileUtils.rm_f(Paths.queue_file)
    end

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

    def fresh_state?(state)
      return false unless state

      updated_at = Time.iso8601(state["updated_at"].to_s)
      dwell = state["dwell_seconds"].to_i
      max_age = dwell.positive? ? dwell + 30 : 90
      Time.now - updated_at <= max_age
    rescue ArgumentError
      false
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

    def stop_pid(target_pid, force_clear:)
      signal_process("TERM", target_pid)
      wait_for_exit(target_pid)
    ensure
      clear_runtime_state if force_clear
    end

    def wait_for_exit(target_pid)
      20.times do
        reap(target_pid)
        break unless process_alive?(target_pid)

        sleep 0.1
      end

      return unless process_alive?(target_pid)

      signal_process_group("KILL", target_pid)
      20.times do
        reap(target_pid)
        break unless process_alive?(target_pid)

        sleep 0.1
      end
    end

    def process_alive?(target_pid)
      return false unless target_pid

      Process.kill(0, target_pid)
      !zombie?(target_pid)
    rescue Errno::ESRCH, Errno::EPERM
      false
    end

    def reap(target_pid)
      Process.waitpid(target_pid, Process::WNOHANG)
    rescue Errno::ECHILD, Errno::ESRCH
      nil
    end

    def zombie?(target_pid)
      status = `ps -o stat= -p #{Integer(target_pid)} 2>/dev/null`.strip
      status.start_with?("Z")
    rescue ArgumentError
      true
    end

    def signal_process_group(signal, target_pid)
      Process.kill(signal, -target_pid)
    rescue Errno::ESRCH
      Process.kill(signal, target_pid)
    end

    def signal_process(signal, target_pid)
      signal_process_group(signal, target_pid)
      Process.kill(signal, target_pid)
    rescue Errno::ESRCH
      nil
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

    def sanitize_log_line(line)
      line.to_s.dup.force_encoding(Encoding::UTF_8).scrub.strip
    end
  end
end
