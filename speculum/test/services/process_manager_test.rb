require "test_helper"
require "tmpdir"

class ProcessManagerTest < ActiveSupport::TestCase
  test "not running when no pidfile exists" do
    Dir.mktmpdir do |runtime_dir|
      runtime_root = Pathname.new(runtime_dir)

      stub_singleton_method(Speculum::Paths, :pidfile, runtime_root.join("speculum.pid")) do
        assert_not Speculum::ProcessManager.new.running?
      end
    end
  end

  test "zombie pid is not treated as running" do
    Dir.mktmpdir do |runtime_dir|
      runtime_root = Pathname.new(runtime_dir)
      pidfile = runtime_root.join("speculum.pid")
      child_pid = Process.spawn(RbConfig.ruby, "-e", "exit!")
      pidfile.write(child_pid.to_s)

      20.times do
        break if `ps -o stat= -p #{child_pid} 2>/dev/null`.strip.start_with?("Z")

        sleep 0.05
      end

      stub_singleton_method(Speculum::Paths, :pidfile, pidfile) do
        assert_not Speculum::ProcessManager.new.running?
        assert_not pidfile.exist?
      end
    ensure
      begin
        Process.waitpid(child_pid) if child_pid
      rescue Errno::ECHILD, Errno::ESRCH
        nil
      end
    end
  end

  test "running player without display state becomes unhealthy after startup window" do
    Dir.mktmpdir do |runtime_dir|
      runtime_root = Pathname.new(runtime_dir)
      pidfile = runtime_root.join("speculum.pid")
      pidfile.write(Process.pid.to_s)
      FileUtils.touch(pidfile, mtime: Time.now - 120)

      stub_singleton_method(Speculum::Paths, :pidfile, pidfile) do
        stub_singleton_method(Speculum::Paths, :state_file, runtime_root.join("player_state.json")) do
          stub_singleton_method(Speculum::Paths, :logfile, runtime_root.join("speculum.log")) do
            assert Speculum::ProcessManager.new.unhealthy?
          end
        end
      end
    end
  end

  test "running player with display state is healthy" do
    Dir.mktmpdir do |runtime_dir|
      runtime_root = Pathname.new(runtime_dir)
      pidfile = runtime_root.join("speculum.pid")
      statefile = runtime_root.join("player_state.json")
      pidfile.write(Process.pid.to_s)
      FileUtils.touch(pidfile, mtime: Time.now - 120)
      statefile.write(JSON.generate("current" => "alpha.png", "updated_at" => Time.now.utc.iso8601, "dwell_seconds" => 60))

      stub_singleton_method(Speculum::Paths, :pidfile, pidfile) do
        stub_singleton_method(Speculum::Paths, :state_file, statefile) do
          assert_not Speculum::ProcessManager.new.unhealthy?
        end
      end
    end
  end

  test "preview shows queued image as next image" do
    Dir.mktmpdir do |project_dir|
      Dir.mktmpdir do |runtime_dir|
        project_root = Pathname.new(project_dir)
        runtime_root = Pathname.new(runtime_dir)
        FileUtils.mkdir_p(project_root.join("IMG"))
        project_root.join("IMG/alpha.png").write("image")
        project_root.join("IMG/bravo.png").write("image")
        runtime_root.join("speculum.log").write("READY\nSending alpha.png...\nDONE\n")
        runtime_root.join("next_image.txt").write("bravo.png")

        stub_singleton_method(Speculum::Paths, :project_root, project_root) do
          stub_singleton_method(Speculum::Paths, :logfile, runtime_root.join("speculum.log")) do
            stub_singleton_method(Speculum::Paths, :queue_file, runtime_root.join("next_image.txt")) do
              settings = Speculum::Settings::DEFAULTS.merge("selected_folder" => "IMG")
              library = Speculum::ImageLibrary.new(settings)
              preview = Speculum::ProcessManager.new.preview(library, settings)

              assert_equal "bravo.png", preview[:next][:name]
            end
          end
        end
      end
    end
  end

  test "preview uses player state file before log inference" do
    Dir.mktmpdir do |project_dir|
      Dir.mktmpdir do |runtime_dir|
        project_root = Pathname.new(project_dir)
        runtime_root = Pathname.new(runtime_dir)
        FileUtils.mkdir_p(project_root.join("IMG"))
        project_root.join("IMG/alpha.png").write("image")
        project_root.join("IMG/bravo.png").write("image")
        runtime_root.join("speculum.log").write("READY\nSending stale.png...\nDONE\n")
        runtime_root.join("player_state.json").write(JSON.generate("current" => "bravo.png", "next" => "alpha.png", "dwell_seconds" => 60, "updated_at" => "2026-06-10T00:00:00Z"))

        stub_singleton_method(Speculum::Paths, :project_root, project_root) do
          stub_singleton_method(Speculum::Paths, :logfile, runtime_root.join("speculum.log")) do
            stub_singleton_method(Speculum::Paths, :state_file, runtime_root.join("player_state.json")) do
              stub_singleton_method(Speculum::Paths, :queue_file, runtime_root.join("next_image.txt")) do
                settings = Speculum::Settings::DEFAULTS.merge("selected_folder" => "IMG")
                library = Speculum::ImageLibrary.new(settings)
                preview = Speculum::ProcessManager.new.preview(library, settings)

                assert_equal "bravo.png", preview[:current][:name]
                assert_equal "alpha.png", preview[:next][:name]
                assert_equal 60, preview[:timer][:duration]
                assert_equal "2026-06-10T00:00:00Z", preview[:timer][:started_at]
              end
            end
          end
        end
      end
    end
  end

  test "preview shows queued image as next over player state" do
    Dir.mktmpdir do |project_dir|
      Dir.mktmpdir do |runtime_dir|
        project_root = Pathname.new(project_dir)
        runtime_root = Pathname.new(runtime_dir)
        FileUtils.mkdir_p(project_root.join("IMG"))
        project_root.join("IMG/alpha.png").write("image")
        project_root.join("IMG/bravo.png").write("image")
        runtime_root.join("player_state.json").write(JSON.generate("current" => "alpha.png", "next" => "alpha.png"))
        runtime_root.join("next_image.txt").write("bravo.png")

        stub_singleton_method(Speculum::Paths, :project_root, project_root) do
          stub_singleton_method(Speculum::Paths, :state_file, runtime_root.join("player_state.json")) do
            stub_singleton_method(Speculum::Paths, :queue_file, runtime_root.join("next_image.txt")) do
              settings = Speculum::Settings::DEFAULTS.merge("selected_folder" => "IMG")
              library = Speculum::ImageLibrary.new(settings)
              preview = Speculum::ProcessManager.new.preview(library, settings)

              assert_equal "alpha.png", preview[:current][:name]
              assert_equal "bravo.png", preview[:next][:name]
            end
          end
        end
      end
    end
  end

  test "preview uses displayed image and countdown before sent image fallback" do
    Dir.mktmpdir do |project_dir|
      Dir.mktmpdir do |runtime_dir|
        project_root = Pathname.new(project_dir)
        runtime_root = Pathname.new(runtime_dir)
        FileUtils.mkdir_p(project_root.join("IMG"))
        project_root.join("IMG/alpha.png").write("image")
        project_root.join("IMG/bravo.png").write("image")
        logfile = runtime_root.join("speculum.log")
        logfile.write("READY\nSending bravo.png...\nDONE\nDisplaying alpha.png for 60 seconds...\n")
        FileUtils.touch(logfile, mtime: Time.utc(2026, 6, 10, 0, 0, 0))

        stub_singleton_method(Speculum::Paths, :project_root, project_root) do
          stub_singleton_method(Speculum::Paths, :runtime_root, runtime_root) do
            stub_singleton_method(Speculum::Paths, :logfile, logfile) do
              settings = Speculum::Settings::DEFAULTS.merge("selected_folder" => "IMG")
              library = Speculum::ImageLibrary.new(settings)
              preview = Speculum::ProcessManager.new.preview(library, settings)

              assert_equal "alpha.png", preview[:current][:name]
              assert_equal "bravo.png", preview[:next][:name]
              assert_equal 60, preview[:timer][:duration]
              assert_equal "2026-06-10T00:00:00Z", preview[:timer][:started_at]
            end
          end
        end
      end
    end
  end

  test "preview falls back to most recent sent image when display line is unavailable" do
    Dir.mktmpdir do |project_dir|
      Dir.mktmpdir do |runtime_dir|
        project_root = Pathname.new(project_dir)
        runtime_root = Pathname.new(runtime_dir)
        FileUtils.mkdir_p(project_root.join("IMG"))
        project_root.join("IMG/alpha.png").write("image")
        project_root.join("IMG/bravo.png").write("image")
        runtime_root.join("speculum.log").write("READY\nSending alpha.png...\nDONE\n")

        stub_singleton_method(Speculum::Paths, :project_root, project_root) do
          stub_singleton_method(Speculum::Paths, :runtime_root, runtime_root) do
            stub_singleton_method(Speculum::Paths, :logfile, runtime_root.join("speculum.log")) do
              settings = Speculum::Settings::DEFAULTS.merge("selected_folder" => "IMG")
              library = Speculum::ImageLibrary.new(settings)
              preview = Speculum::ProcessManager.new.preview(library, settings)

              assert_equal "alpha.png", preview[:current][:name]
              assert_equal "bravo.png", preview[:next][:name]
            end
          end
        end
      end
    end
  end

  test "recent log tails large log files without reading unrelated old entries" do
    Dir.mktmpdir do |runtime_dir|
      runtime_root = Pathname.new(runtime_dir)
      logfile = runtime_root.join("speculum.log")
      logfile.write((["old"] * 20_000 + ["Sending recent.png...", "DONE"]).join("\n"))

      stub_singleton_method(Speculum::Paths, :logfile, logfile) do
        lines = Speculum::ProcessManager.new.recent_log(lines: 2)

        assert_equal ["Sending recent.png...", "DONE"], lines
      end
    end
  end
end
