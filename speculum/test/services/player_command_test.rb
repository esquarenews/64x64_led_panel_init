require "test_helper"
require "tmpdir"

class PlayerCommandTest < ActiveSupport::TestCase
  test "builds serial command with auto port and reset pulse" do
    settings = Speculum::Settings::DEFAULTS.merge(
      "selected_folder" => "IMG",
      "transport" => "serial",
      "port" => "auto",
      "hard_reset_before_start" => "1",
      "overlay" => "0",
      "calendar" => "0"
    )

    argv = Speculum::PlayerCommand.new(settings).argv

    assert_includes argv, "--transport"
    assert_includes argv, "serial"
    assert_includes argv, "--port"
    assert_includes argv, "auto"
    assert_includes argv, "--hard-reset"
    assert_includes argv, "--no-overlay"
    assert_includes argv, "--no-calendar"
    assert_includes argv, "--queue-file"
    assert_includes argv, "--state-file"
  end

  test "single image mode creates a runtime directory with only the chosen image" do
    Dir.mktmpdir do |project_dir|
      Dir.mktmpdir do |storage_dir|
        project_root = Pathname.new(project_dir)
        storage_root = Pathname.new(storage_dir)
        FileUtils.mkdir_p(project_root.join("IMG"))
        source = project_root.join("IMG/panel.png")
        source.write("png")

        settings = Speculum::Settings::DEFAULTS.merge(
          "mode" => "single",
          "selected_folder" => "IMG",
          "selected_image" => "panel.png"
        )

        stub_singleton_method(Speculum::Paths, :project_root, project_root) do
          stub_singleton_method(Speculum::Paths, :storage_root, storage_root) do
            stub_singleton_method(Speculum::Paths, :runtime_root, storage_root.join("runtime")) do
              stub_singleton_method(Speculum::Paths, :single_image_dir, storage_root.join("runtime/single_image")) do
                command = Speculum::PlayerCommand.new(settings)

                assert_equal storage_root.join("runtime/single_image"), command.image_source_dir
                assert_path_exists storage_root.join("runtime/single_image/panel.png")
                assert_includes command.argv, "--single"
                assert_not_includes command.argv, "--queue-file"
                assert_includes command.argv, "--state-file"
              end
            end
          end
        end
      end
    end
  end
end
