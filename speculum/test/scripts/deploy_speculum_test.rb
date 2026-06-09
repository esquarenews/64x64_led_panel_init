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
  end
end
