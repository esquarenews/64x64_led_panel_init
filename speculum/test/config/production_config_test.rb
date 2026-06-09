require "open3"
require "test_helper"

class ProductionConfigTest < ActiveSupport::TestCase
  test "production is ready for the public caddy hostname" do
    script = <<~RUBY
      require_relative "config/environment"
      config = Rails.application.config
      abort "missing host" unless config.hosts.include?("speculum.esquarenews.tech")
      abort "missing localhost health host" unless config.hosts.include?("127.0.0.1")
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

  test "production resolves the application stylesheet asset" do
    script = <<~RUBY
      require_relative "config/environment"
      helper = ActionController::Base.helpers
      path = helper.stylesheet_path("application")
      abort path unless path.include?("/assets/application-") && path.end_with?(".css")
    RUBY

    env = {
      "RAILS_ENV" => "production",
      "SECRET_KEY_BASE_DUMMY" => "1",
      "SPECULUM_USERNAME" => "admin",
      "SPECULUM_PASSWORD" => "speculum"
    }

    _stdout, stderr, status = Open3.capture3(env, RbConfig.ruby, "-e", script, chdir: Rails.root)

    assert status.success?, stderr
    assert_includes Rails.root.join("app/views/layouts/application.html.erb").read, 'stylesheet_link_tag "application"'
    assert_not_includes Rails.root.join("app/views/layouts/application.html.erb").read, "stylesheet_link_tag :app"
  end

  test "production resolves loading indicator assets" do
    script = <<~RUBY
      require_relative "config/environment"
      helper = ActionController::Base.helpers
      path = helper.javascript_path("application")
      abort path unless path.include?("/assets/application-") && path.end_with?(".js")
    RUBY

    env = {
      "RAILS_ENV" => "production",
      "SECRET_KEY_BASE_DUMMY" => "1",
      "SPECULUM_USERNAME" => "admin",
      "SPECULUM_PASSWORD" => "speculum"
    }

    _stdout, stderr, status = Open3.capture3(env, RbConfig.ruby, "-e", script, chdir: Rails.root)

    layout = Rails.root.join("app/views/layouts/application.html.erb").read
    javascript = Rails.root.join("app/assets/javascripts/application.js").read
    stylesheet = Rails.root.join("app/assets/stylesheets/application.css").read

    assert status.success?, stderr
    assert_includes layout, 'javascript_include_tag "application"'
    assert_includes javascript, "form.classList.add(\"is-submitting\")"
    assert_includes javascript, "submitAfterPaint(form)"
    assert_includes javascript, "loading-overlay"
    assert_includes javascript, "Deleting..."
    assert_includes stylesheet, "button.is-submitting"
    assert_includes stylesheet, ".loading-overlay"
  end

  test "logo assets are wired for branding and favicon" do
    layout = Rails.root.join("app/views/layouts/application.html.erb").read
    dashboard = Rails.root.join("app/views/dashboard/show.html.erb").read
    login = Rails.root.join("app/views/sessions/new.html.erb").read

    assert_path_exists Rails.root.join("app/assets/images/speculum-logo.png")
    assert_path_exists Rails.root.join("public/icon.png")
    assert_includes layout, 'href="/icon.png"'
    assert_not_includes layout, 'href="/icon.svg"'
    assert_includes dashboard, "speculum-logo.png"
    assert_includes login, "speculum-logo.png"
  end

  test "settings toggles use switch styling" do
    dashboard = Rails.root.join("app/views/dashboard/show.html.erb").read
    stylesheet = Rails.root.join("app/assets/stylesheets/application.css").read

    assert_includes dashboard, 'class="toggle"'
    assert_includes dashboard, '<i aria-hidden="true"></i>'
    assert_includes stylesheet, ".toggle input:checked + i"
    assert_includes stylesheet, "white-space: nowrap"
  end

  test "preview countdown assets are wired" do
    dashboard = Rails.root.join("app/views/dashboard/show.html.erb").read
    javascript = Rails.root.join("app/assets/javascripts/application.js").read
    stylesheet = Rails.root.join("app/assets/stylesheets/application.css").read

    assert_includes dashboard, "data-countdown-started-at"
    assert_includes dashboard, "Changes in"
    assert_includes dashboard, "Up in"
    assert_includes javascript, "updateCountdowns"
    assert_includes stylesheet, ".preview-timer"
  end

  test "upload file input has a wide layout" do
    stylesheet = Rails.root.join("app/assets/stylesheets/application.css").read

    assert_includes stylesheet, "flex: 0 1 520px"
    assert_includes stylesheet, "min-width: 280px"
    assert_not_includes stylesheet, "max-width: 210px"
  end
end
