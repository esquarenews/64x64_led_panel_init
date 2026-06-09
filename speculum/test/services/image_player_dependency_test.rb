require "bundler"
require "test_helper"

class ImagePlayerDependencyTest < ActiveSupport::TestCase
  test "production bundle includes image player fallback gems" do
    specs = Bundler.load.specs.map(&:name)

    assert_includes specs, "serialport"
    assert_includes specs, "mini_magick"
    assert_includes specs, "chunky_png"
  end
end
