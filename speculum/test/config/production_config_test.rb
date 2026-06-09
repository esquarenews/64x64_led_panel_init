require "open3"
require "test_helper"

class ProductionConfigTest < ActiveSupport::TestCase
  test "production is ready for the public caddy hostname" do
    script = <<~RUBY
      require_relative "config/environment"
      config = Rails.application.config
      abort "missing host" unless config.hosts.include?("speculum.esquarenews.tech")
      abort "assume_ssl disabled" unless config.assume_ssl
    RUBY

    env = {
      "RAILS_ENV" => "production",
      "SECRET_KEY_BASE_DUMMY" => "1",
      "SPECULUM_USERNAME" => "admin",
      "SPECULUM_PASSWORD" => "speculum"
    }

    _stdout, stderr, status = Open3.capture3(env, RbConfig.ruby, "-e", script, chdir: Rails.root)

    assert status.success?, stderr
  end
end
