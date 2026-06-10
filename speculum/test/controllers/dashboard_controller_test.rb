require "test_helper"
require "tmpdir"

class DashboardControllerTest < ActionDispatch::IntegrationTest
  FakePlayer = Struct.new(:running, :restarted_with, :preview_result) do
    def running?
      running
    end

    def restart(settings)
      self.restarted_with = settings
    end

    def preview(_library, _settings)
      preview_result || {}
    end
  end

  FakeResetter = Struct.new(:called) do
    def reset_all
      self.called = true
    end
  end

  test "changing folder restarts running player with hard reset" do
    with_dashboard_project do |project_root, storage_root|
      player = FakePlayer.new(true)

      stub_singleton_method(Speculum::ProcessManager, :new, player) do
        patch settings_path, params: { settings: { selected_folder: "IMG2", mode: "loop" } }
      end

      settings = YAML.safe_load(storage_root.join("speculum_settings.yml").read)

      assert_redirected_to root_path
      assert_equal "IMG2", settings["selected_folder"]
      assert_equal "IMG2", player.restarted_with["selected_folder"]
      assert_equal "1", player.restarted_with["hard_reset_before_start"]
    end
  end

  test "saving same folder does not restart running player" do
    with_dashboard_project do |_project_root, _storage_root|
      player = FakePlayer.new(true)

      stub_singleton_method(Speculum::ProcessManager, :new, player) do
        patch settings_path, params: { settings: { selected_folder: "IMG", mode: "loop" } }
      end

      assert_redirected_to root_path
      assert_nil player.restarted_with
    end
  end

  test "hard reset restarts running player" do
    with_dashboard_project do |_project_root, _storage_root|
      player = FakePlayer.new(true)
      resetter = FakeResetter.new(false)

      stub_singleton_method(Speculum::ProcessManager, :new, player) do
        stub_singleton_method(Speculum::PortResetter, :new, resetter) do
          post reset_player_path
        end
      end

      assert_redirected_to root_path
      assert resetter.called
      assert_equal "1", player.restarted_with["hard_reset_before_start"]
    end
  end

  test "hard reset starts player when it was paused" do
    with_dashboard_project do |_project_root, _storage_root|
      player = FakePlayer.new(false)
      resetter = FakeResetter.new(false)

      stub_singleton_method(Speculum::ProcessManager, :new, player) do
        stub_singleton_method(Speculum::PortResetter, :new, resetter) do
          post reset_player_path
        end
      end

      assert_redirected_to root_path
      assert resetter.called
      assert_equal "1", player.restarted_with["hard_reset_before_start"]
    end
  end

  test "preview returns current and next image json" do
    with_dashboard_project do |_project_root, _storage_root|
      player = FakePlayer.new(true)
      player.preview_result = {
        current: { name: "a.png", thumbnail_url: "/images/thumbnail?folder=IMG&name=a.png" },
        next: { name: "b.png", thumbnail_url: "/images/thumbnail?folder=IMG&name=b.png" },
        timer: { started_at: "2026-06-10T00:00:00Z", duration: 60 }
      }

      stub_singleton_method(Speculum::ProcessManager, :new, player) do
        get preview_player_path
      end

      assert_response :success
      payload = JSON.parse(response.body)
      assert_equal true, payload["running"]
      assert_equal "a.png", payload.dig("preview", "current", "name")
      assert_equal "b.png", payload.dig("preview", "next", "name")
      assert_equal 60, payload.dig("preview", "timer", "duration")
    end
  end

  private

  def with_dashboard_project
    Dir.mktmpdir do |project_dir|
      Dir.mktmpdir do |storage_dir|
        project_root = Pathname.new(project_dir)
        storage_root = Pathname.new(storage_dir)
        FileUtils.mkdir_p(project_root.join("IMG"))
        FileUtils.mkdir_p(project_root.join("IMG2"))
        project_root.join("IMG/a.png").write("image")
        project_root.join("IMG2/b.png").write("image")

        stub_singleton_method(Speculum::Paths, :project_root, project_root) do
          stub_singleton_method(Speculum::Paths, :storage_root, storage_root) do
            post session_path, params: { username: "admin", password: "speculum" }
            yield project_root, storage_root
          end
        end
      end
    end
  end
end
