require "open3"
require "test_helper"

class DeploySpeculumTest < ActiveSupport::TestCase
  test "deploy script is valid bash" do
    script = Rails.root.parent.join("scripts/deploy_speculum.sh")

    _stdout, stderr, status = Open3.capture3("bash", "-n", script.to_s)

    assert status.success?, stderr
  end

  test "deploy script installs a production systemd service" do
    script = Rails.root.parent.join("scripts/deploy_speculum.sh").read

    assert_includes script, "git pull --ff-only origin"
    assert_includes script, "bundle install"
    assert_includes script, "rails assets:precompile"
    assert_includes script, "systemctl restart"
    assert_includes script, "bundle exec rails server"
    assert_includes script, "EnvironmentFile=$ENV_FILE"
    assert_includes script, "Environment=PATH=$SERVICE_PATH"
    assert_includes script, "PORT=\"${PORT:-5000}\""
    assert_includes script, "update_runtime_env"
    assert_includes script, "RESET_CREDENTIALS"
    assert_includes script, "SPECULUM_PASSWORD"
  end
end
