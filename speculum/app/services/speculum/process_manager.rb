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

    def recent_log(lines: 12)
      return [] unless Paths.logfile.exist?

      tail_lines(Paths.logfile, lines).map(&:strip)
    end

    def preview(library, settings)
      image_names = library.images(settings["selected_folder"]).map { |image| image[:name] }
      current = current_image_name(image_names) || image_names.first
      queued = queued_image_name(image_names)
      next_image = queued || next_image_name(image_names, current)
      {
        current: current && library.images(settings["selected_folder"]).find { |image| image[:name] == current },
        next: next_image && library.images(settings["selected_folder"]).find { |image| image[:name] == next_image }
      }
    end

    private

    def wait_for_exit
      20.times do
        break unless running?
        sleep 0.1
      end
      Process.kill("KILL", -pid) if running?
    end

    def current_image_name(image_names)
      recent_log(lines: 100).reverse_each do |line|
        if (match = line.match(/\ASending (.+)\.\.\.\z/))
          name = match[1]
          return name if image_names.include?(name)
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

    def queued_image_name(image_names)
      return unless Paths.queue_file.exist?

      name = Paths.queue_file.read.strip
      image_names.include?(name) ? name : nil
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
