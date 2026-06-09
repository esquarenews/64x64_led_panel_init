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

  test "preview uses the most recent sent image and next image" do
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
end
